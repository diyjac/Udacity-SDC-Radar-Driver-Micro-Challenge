#!/usr/bin/python
"""
esr_visualizer.py: version 0.1.0

Todo:
convert rosbag can_raw ESR track data to image

History:
2016/10/28: Initial version to display visual radar data from ros topic 'esr_front'.
"""

import math
import numpy as np
import argparse
import sys
import numpy as np
import rospy
import datetime
import struct
import json
import std_msgs

try:
    import cv2
except ImportError:
    print "Error importing opencv"
    pass

'''
IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3
'''

class RadarVisualizer(object):
    def __init__(self, width=250, height=250, channels=3):
        self.width = width
        self.height = height
        self.channels = channels
        self.font = cv2.FONT_HERSHEY_SIMPLEX 
        self.img = np.zeros((self.height, self.width, self.channels), np.uint8)
        cv2.imshow("Radar", self.img)
        pass

    def update(self, radarData):
        self.img = np.zeros((self.height, self.width, self.channels), np.uint8)
        cv2.line(self.img, (10, 0), (self.width/2 - 5, self.height), (100, 255, 255))
        cv2.line(self.img, (self.width - 10, 0), (self.width/2 + 5, self.height), (100, 255, 255))

        for track_number in range(1, 65):
            if str(track_number)+'_track_range' in radarData:
                track_range = radarData[str(track_number)+'_track_range']
                track_angle = (float(radarData[str(track_number)+'_track_angle'])+90.0)*math.pi/180

                x_pos = math.cos(track_angle)*track_range*4
                y_pos = math.sin(track_angle)*track_range*4

                cv2.circle(self.img, (self.width/2 + int(x_pos), self.height - int(y_pos) - 10), 5, (255, 255, 255))
                #cv2.putText(self.img, str(track_number), 
                #    (self.width/2 + int(x_pos)-2, self.height - int(y_pos) - 10), self.font, 1, (255,255,255), 2)

        cv2.imshow("Radar", self.img)
        cv2.waitKey(2)

visualizer = RadarVisualizer()
def callback(data):
    print "data: ", data
    object = json.loads(data.data)
    visualizer.update(object)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Udacity SDC Micro Challenge Radar viewer')
    parser.add_argument('--debug', action='store_true', default=False, help='display debug messages')
    args = parser.parse_args()
    debug = args.debug

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ros_esr_visualizer', anonymous=True)
    rospy.Subscriber("esr_front", std_msgs.msg.String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

