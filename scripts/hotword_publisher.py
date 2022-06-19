#!/usr/bin/env python3

import rospy
import rospkg
import sys
import os
import glob
from pathlib import Path
sys.path.insert(1, os.getenv('SNOWBOY_PYTHON3_PATH')) # define SNOWBOY_PYTHON3_PATH in ~/.bashrc with "export SNOWBOY_PYTHON3_PATH = /snowboy/examples/Python3"
import snowboydecoder
from std_msgs.msg import String


class HotwordPublisher:
    def __init__(self):
        # ROS VARIABLES
        rospy.init_node('hotword_publisher')                                 # initialise rospy node
        pkg_path = rospkg.RosPack().get_path("snowboy_ros")                  # path to snowboy_ros package
        self.pub = rospy.Publisher("cmd_msgs", String, queue_size = 10)      # publish string msgs corresponding to hotword commands
        # SNOWBOY VARIABLES
        model_path = f"{pkg_path}/scripts/models"                            # "models" dir path
        self.models = glob.glob(f"{model_path}/*.pmdl")                      # list of hotword model paths (e.g. "~/models/model.pmdl")
        cmd_msgs = [Path(m).stem for m in self.models]                       # command msgs to publish (turns "~/models/spot_sit.pmdl" to "spot_sit")
        self.callbacks = [lambda i=i: self.pub.publish(i) for i in cmd_msgs] # callbacks per model, publishes corresponding string msg
        
    def detect(self):
        # Hotword Detector: uses hotword models from "models" dir as inputs
        detector = snowboydecoder.HotwordDetector(self.models, sensitivity=0.5, audio_gain=1)
        # Starts detector: when a hotword is detected, call the corresponding callback function. Stop if ROS shuts down
        detector.start(detected_callback=self.callbacks, interrupt_check=lambda: rospy.is_shutdown())

if __name__ == '__main__':
    hw_publisher = HotwordPublisher()
    hw_publisher.detect()
    
