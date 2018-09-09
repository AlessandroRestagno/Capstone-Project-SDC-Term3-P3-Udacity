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

def infer():
    num_classes = 4
    base_model = ResNet50(weights=None, include_top=False)
    x = base_model.output
    x = GlobalAveragePooling2D()(x)
    x = Dense(1024, activation="relu")(x)
    predictions = Dense(num_classes, activation="softmax")(x)
    model_final = Model(inputs = base_model.input, outputs = predictions)
    preprocess = res_preprocess
    image_size = (224, 224)
    
    import tensorflow as tf
    sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))
    
    input_folder="/home/workspace/Capstone-Project-SDC-Term3-P3-Udacity/train/examples/green/"
    image_paths = glob.glob(os.path.join(input_folder, "*.jpg"))
    shuffle(image_paths)
    
    model_final.load_weights("resnet50_very_good.h5")

    start_time = datetime.datetime.now()
    nr_images = 200
    
    labels_dict = {'green': 0, 'no': 1, 'orange': 2, 'red': 3}
    labels = ['green', 'no', 'orange', 'red']
    print(labels)
    
    bad_samples = 0
    for i, image_name in enumerate(image_paths[:nr_images]):
        im = cv2.imread(image_name)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        im = im.astype(np.float64)
        im = res_preprocess(im)
        im = np.expand_dims(im, axis =0)
        prob = model_final.predict(im)
        probs = prob[0]
        print(probs)
        j = np.argmax(prob,axis=1)[0]
#         img=mpimg.imread(image_name)
#         axes[i].imshow(img)
        if j == 3:
            bad_samples += 1
        print('Image name: ' + image_name)
        print('Best guess: %s with certainty %f' % (labels[j],probs[j]))
    
    end_time = datetime.datetime.now()
    time_diff = end_time - start_time
    print ("Time to run: " + str(time_diff))
    print ("Bad samples %: " + str((float)(bad_samples) * 100 / nr_images))
    
if __name__ == "__main__":
    infer()