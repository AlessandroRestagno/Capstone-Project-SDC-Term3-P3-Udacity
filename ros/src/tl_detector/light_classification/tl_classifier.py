from styx_msgs.msg import TrafficLight

from keras.applications.resnet50 import ResNet50, preprocess_input as res_preprocess
from keras.models import Model
from keras.layers import Input, Dense, GlobalAveragePooling2D
import numpy as np
import glob
import cv2
import h5py
import os
import json
import datetime
import time
from random import shuffle
import tensorflow as tf
import rospy

CLASSIFICATION_PROB_THRESHOLD = 0.90

class TLClassifier(object):
    def __init__(self):
        num_classes = 4
        base_model = ResNet50(weights=None, include_top=False)
        x = base_model.output
        x = GlobalAveragePooling2D()(x)
        x = Dense(1024, activation="relu")(x)
        predictions = Dense(num_classes, activation="softmax")(x)
        self.model_final = Model(inputs = base_model.input, outputs = predictions)

        sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))

        cwd = os.path.dirname(os.path.realpath(__file__))
        rospy.loginfo('TLC cwd %s', cwd)
        self.model_final.load_weights(cwd + '/resnet50_5.h5')

        self.graph = tf.get_default_graph()
        
        self.labels_dict = {'green': 0, 'no': 1, 'orange': 2, 'red': 3}
        self.labels = ['green', 'no', 'orange', 'red']

        self.model_loaded = 1
        rospy.loginfo("TLC Classification model (sim) loaded.")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        start_time = datetime.datetime.now()

        # preprocessing image
        im = image
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        im = im.astype(np.float64)
        image_size = (224, 224)
        im = cv2.resize(im, image_size, interpolation = cv2.INTER_CUBIC)
        im = res_preprocess(im)
        im = np.expand_dims(im, axis =0)

        # predict traffic light state
        if self.model_loaded:
            with self.graph.as_default():
                prob = self.model_final.predict(im)
                probs = prob[0]
                j = np.argmax(prob, axis=1)[0]

                end_time = datetime.datetime.now()
                time_diff = end_time - start_time

                #rospy.loginfo('Best guess: %s with certainty %f' % (self.labels[j], probs[j]))
                #rospy.loginfo('Time to run: ' + str(time_diff))

                if probs[j] > CLASSIFICATION_PROB_THRESHOLD:
                    if j == 0:
                        #rospy.loginfo('Get classification function - Returning green.')
                        return TrafficLight.GREEN
                        
                    elif j == 1:
                        #rospy.loginfo('Get classification function - Returning no.')
                        return TrafficLight.UNKNOWN
                    elif j == 2:
                        #rospy.loginfo('Get classification function - Returning yellow.')
                        return TrafficLight.YELLOW
                    elif j == 3:
                        #rospy.loginfo('Get classification function - Returning red.')
                        return TrafficLight.RED

        return TrafficLight.UNKNOWN