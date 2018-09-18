#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_classifiers_site import TLClassifierSite
from scipy.spatial import KDTree

import datetime
import json

import tf
import cv2
import yaml
import os

STATE_COUNT_THRESHOLD = 2
SKIP_FRAMES = 2 # number of frames skipped in classification to ensure real time capability
CLASSIFICATION_DIST_THRESHOLD = 175 # distance threshold below of that, camera image will be classified


def join_files(origFileName, newFileName, noOfChunks):
    dataList = []

    j = 0
    for i in range(0, noOfChunks, 1):
        j += 1
        chunkName = "%s-chunk-%s-Of-%s" % (origFileName, j, noOfChunks)
        f = open(chunkName, 'rb')
        dataList.append(f.read())
        f.close()

    j = 0
    for i in range(0, noOfChunks, 1):
        j += 1
        chunkName = "%s-chunk-%s-Of-%s" % (origFileName, j, noOfChunks)
        # Deleting the chunck file.
        os.remove(chunkName)

    f2 = open(newFileName, 'wb')
    for data in dataList:
        f2.write(data)
    f2.close()


def join_model_files():
    cwd = os.path.dirname(os.path.realpath(__file__))
    resnet_model = os.path.join(cwd, 'light_classification', 'resnet50_5.h5')
    vgg19_model = os.path.join(cwd, 'light_classification', 'vgg19_site.h5')

    if not os.path.isfile(resnet_model):
        join_files(origFileName=resnet_model, newFileName=resnet_model, noOfChunks=3)
    if not os.path.isfile(vgg19_model):
        join_files(origFileName=vgg19_model, newFileName=vgg19_model, noOfChunks=2)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.init_finished = 0
        self.pose = None
        self.waypoints_2d = None
        self.camera_image = None
        self.waypoint_tree = None
        self.lights = []

        join_model_files()

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        print(self.config)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        
        if self.config['is_site']:
            print('Test')
            self.light_classifier = TLClassifierSite()
        else:
            self.light_classifier = TLClassifier()
                
        self.listener = tf.TransformListener()

        self.frame_count = 0

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
    
        self.init_finished = 1
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        #rospy.loginfo('Image cb called.')
        if self.init_finished:
            if self.frame_count >= SKIP_FRAMES:

                self.frame_count = 0

                light_wp, state = self.process_traffic_lights()

                '''
                Publish upcoming red lights at camera frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times till we start using it. Otherwise the previous stable state is
                used.
                '''
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state
                    light_wp = light_wp if state == TrafficLight.RED else -1
                    self.last_wp = light_wp
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                    #rospy.loginfo('State verified and now will be published.')
                    #rospy.loginfo('Index waypoint: %d', light_wp)
                    #rospy.loginfo('State of light: %s' ,state)
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))

                self.state_count += 1

            self.frame_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # get classification
        return self.light_classifier.get_classification(cv_image)

        # for testing reasons you can use ground truth data
        #rospy.loginfo('Image classified. State of light is %s', light.state)
        #return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # rospy.loginfo('Process image.')
        
        # list of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            #rospy.loginfo('Car is next to wp: %d', car_wp_idx)
            
            
            # find the closest visible traffic light (if one exists)
            diff = len(self.base_waypoints.waypoints)
            light_idx = -1
            for i,light in enumerate (self.lights):
                # get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # find closest stop line waypoint index
                d = (temp_wp_idx - car_wp_idx) % len(self.base_waypoints.waypoints)
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
                    light_idx = i

            # debug output
            #rospy.loginfo('Closest light has state: %d', closest_light.state)
            #rospy.loginfo('Index diff car to next light: %d', diff)
            #rospy.loginfo('Closest light has index: %d', light_idx)
            #rospy.loginfo('Len base waypoints: %d', len(self.base_waypoints.waypoints))

        # save images, current pose and state if recording
        if self.config['recording']:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            now = datetime.datetime.now()
            file_suffix = now.isoformat()
            img_name = 'data-'+file_suffix+'.jpg'
            cv2.imwrite(img_name, cv_image)

            json_data = {
                "pose": yaml.load(str(self.pose)),
                "lights": [yaml.load(str(light)) for light in self.lights],
                "closest_light": yaml.load(str(closest_light)) if not None else {},
                "image_name": img_name,
                "diff": diff,
                "light_index": light_idx,
                "waypoint_index": line_wp_idx
            }

            json_file_name = 'data-'+file_suffix+'.json'
            with open(json_file_name, 'w') as outfile:
                json.dump(json_data, outfile)

        if closest_light and diff <= CLASSIFICATION_DIST_THRESHOLD:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
