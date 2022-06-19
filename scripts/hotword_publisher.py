#!/usr/bin/env python3
import rospy
import rospkg
from std_msgs.msg import String
import glob
from pathlib import Path
from os import getenv
import sys
sys.path.insert(1, getenv('SNOWBOY_PYTHON3_PATH')) # includes path to snowboydecoder.py defined in "~/.bashrc"
import snowboydecoder


class HotwordPublisher:
    def __init__(self):
        # ROS VARIABLES
        rospy.init_node('hotword_publisher')                               # initialise rospy node
        pkg_path = rospkg.RosPack().get_path("snowboy_ros")                # path to snowboy_ros package
        pub = rospy.Publisher("cmd_msgs", String, queue_size = 10)         # string message publisher
        # SNOWBOY VARIABLES
        model_path = f"{pkg_path}/scripts/models"                          # "models" directory path
        self.__models = glob.glob(f"{model_path}/*.pmdl")                  # list of hotword model paths (e.g. "~/models/model.pmdl")
        str_msgs = [Path(m).stem for m in self.__models]                   # list of string msgs per model (turns "~/models/spot_sit.pmdl" to "spot_sit")
        self.__callbacks = [lambda i=i: pub.publish(i) for i in str_msgs]  # list of callbacks per model: publishes corresponding string msg
        
    def detect(self):
        # Hotword Detector: uses hotword models from "models" dir as inputs
        detector = snowboydecoder.HotwordDetector(self.__models, sensitivity=0.5, audio_gain=1)
        # Start the detector. When a hotword is detected, it calls the corresponding callback function
        detector.start(detected_callback=self.__callbacks, interrupt_check=lambda: rospy.is_shutdown())


if __name__ == '__main__':
    hw_publisher = HotwordPublisher()
    hw_publisher.detect()