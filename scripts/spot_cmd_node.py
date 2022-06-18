#!/usr/bin/env python3

import rospy
import sys
import os
sys.path.insert(1, os.getenv('SNOWBOY_PYTHON3_PATH')) # define SNOWBOY_PYTHON3_PATH in ~/.bashrc with "export SNOWBOY_PYTHON3_PATH = /snowboy/examples/Python3"
import snowboydecoder
from std_msgs.msg import String

CATKIN_WS_PATH = os.getenv('CATKIN_WS_PATH') # define CATKIN_WS_PATH in ~/.bashrc with "export CATKIN_WS_PATH = /"your catkin_ws directory"/src"

class SpotCmdDetector:
    def sit_callback(self):
        self.message = "sit"
        self.pub.publish(self.message)
        
    def stand_callback(self):
        self.message = "stand"
        self.pub.publish(self.message)

    def down_callback(self):
        self.message = "down"
        self.pub.publish(self.message)
        
    def __init__(self):
        # ROS VARIABLES
        rospy.init_node('spot_cmds')
        self.message = ""
        self.pub = rospy.Publisher("spot_cmds", String, queue_size = 10)
        # SNOWBOY VARIABLES
        path = f"{CATKIN_WS_PATH}/snowboy_ros/scripts/models"
        self.models = [f"{path}/spot_sit.pmdl", f"{path}/spot_stand.pmdl", f"{path}/spot_down.pmdl"]
        self.callbacks = [self.sit_callback, self.stand_callback, self.down_callback]
                
    def detect(self):
        detector = snowboydecoder.HotwordDetector(self.models, sensitivity=0.5, audio_gain=1)
        detector.start(detected_callback=self.callbacks, interrupt_check=lambda: rospy.is_shutdown())

if __name__ == '__main__':
    spot_cmdr = SpotCmdDetector()
    spot_cmdr.detect()
    