#!/usr/bin/python
"""
view_rosbag_radar.py: version 0.1.0

Todo:
convert rosbag can_raw ESR track data to image

History:
2016/10/02: Initial version to display raw radar data.
"""

import argparse
import sys
import numpy as np
import pygame
import rosbag
import datetime
import struct

#from keras.models import model_from_json

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Udacity SDC Micro Challenge Radar viewer')
  parser.add_argument('--dataset', type=str, default="dataset.bag", help='Dataset/ROS Bag name')
  parser.add_argument('--skip', type=int, default="0", help='skip seconds')
  args = parser.parse_args()

  dataset = args.dataset
  skip = args.skip
  startsec = 0

  print "reading rosbag ", dataset
  bag = rosbag.Bag(dataset, 'r')

#
# msg format:
#   header: 
#     seq: 120556
#     stamp: 
#       secs: 1476173337
#       nsecs: 165327047
#     frame_id: ''
#   count: 120556
#   id: 1899
#   len: 8
#   dat: [2, 0, 0, 0, 0, 0, 0, 0]
#   flag: 0
#   time: 1219812

  for topic, msg, t in bag.read_messages(topics=['/can_raw']):
    if startsec == 0:
        startsec = t.to_sec()
        if skip < 24*60*60:
            skipping = t.to_sec() + skip
            print "skipping ", skip, " seconds from ", startsec, " to ", skipping, " ..."
        else:
            skipping = skip
            print "skipping to ", skip, " from ", startsec, " ..."
    else:
        if t.to_sec() > skipping:
            if topic in ['/can_raw']:
                sys.stdout.write(topic)
                sys.stdout.write(" seq:%d " % msg.header.seq)
                sys.stdout.write(" id:0x%0.4x " % msg.id)
                sys.stdout.write(" msglen:%d " % msg.len)
                for i in xrange(msg.len):
                    sys.stdout.write(" 0x%0.2x " % struct.unpack('B', msg.dat[i])[0])
                sys.stdout.write("\n")

