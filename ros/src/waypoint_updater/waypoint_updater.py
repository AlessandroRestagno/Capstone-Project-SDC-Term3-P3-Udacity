#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math
import datetime

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 35 #20 # 100 # Number of waypoints we will publish. You can change this number, was: 200
LOOP_FREQ = 5 #2 # 5 # was: 50
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('init')

        # other member variables
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.closest_idx = -1
        self.curspeed = -1

        # Add a subscribers for /traffic_waypoint and /obstacle_waypoint
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        # Final waypoints publisher
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        # rospy.loginfo('loop')

        rate = rospy.Rate(LOOP_FREQ)
        while not rospy.is_shutdown():
            # rospy.loginfo('cycle')
            if self.pose and self.base_lane and self.waypoint_tree:
                # Get closest waypoint
                # closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
        # rospy.loginfo('publish_wp')

        # lane = Lane()
        # lane.header = self.base_lane.header # probably not really needed
        # lane.waypoints = self.base_lane.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        # self.final_waypoints_pub.publish(lane)
        
        #start_time = datetime.datetime.now()
        
        final_lane = self.generate_lane()
        
        #end_time = datetime.datetime.now()
        #time_diff = end_time - start_time

        # rospy.loginfo('Best guess: %s with certainty %f' % (self.labels[j], probs[j]))
        #rospy.loginfo('Time to run: ' + str(time_diff))
        
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_lane.header # probably not really needed

        closest_idx = self.get_closest_waypoint_idx()
        self.closest_idx = closest_idx

        # rospy.logdebug('closest_idx %d', closest_idx)
        # rospy.loginfo('closest_idx %d', closest_idx)

        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[(closest_idx + 4) : (farthest_idx + 4)] # slicing

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
            #rospy.loginfo('Normal lane waypoints')
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            #rospy.loginfo('Stop lane waypoints')

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        """
        Decelerate waypoints.

        Args:
          waypoints: list of (sliced) waypoints
          closest_idx: closest waypoint index of self

        Returns:
          new list of decelerated waypoints
        """
        # determine waypoint index for stopping in front of red light stopline
        stop_idx = max(self.stopline_wp_idx - closest_idx - 4, 0)
                
        # if self.distance(waypoints, closest_idx, stop_idx) > 50:
        #    return waypoints
        
        temp = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
            
            if i == 0:
                self.curspeed = p.twist.twist.linear.x

        return temp

    def pose_cb(self, msg):
        # rospy.logdebug('pose_cb')
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # rospy.loginfo('WaypointUpdater waypoint_cb')
        self.base_lane = waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
                                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        """
        Callback for /traffic_waypoint message.
        """
        self.stopline_wp_idx = msg.data
        rospy.loginfo('traffic_cb nextTL: %d curIDX: %d spd: %d', self.stopline_wp_idx, self.closest_idx, self.curspeed)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
