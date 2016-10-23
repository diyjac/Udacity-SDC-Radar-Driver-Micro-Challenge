### Requirement
Need to install
+ [ROS Indigo] (http://wiki.ros.org/indigo/Installation/Ubuntu)
+ [Kvaser LINUX Driver and SDK] (https://www.kvaser.com/downloads/)

### Build ROS package
cd to Udacity-SDC-Radar-Driver-Micro-Challenge/ros
```
source /opt/ros/indigo/setup.bash
catkin_make
source devel/setup.bash
```

### Run ROS package
```
rosrun kvaser esr_listener.py -h
usage: esr_listener.py [-h] [--channel CHANNEL] [--dataset DATASET] [--debug]
                       [--mode MODE] [--skip SKIP] [--topic TOPIC] [--visual]

Udacity SDC Micro Challenge Radar viewer

optional arguments:
  -h, --help         show this help message and exit
  --channel CHANNEL  CANbus channel where target ESR radar is located
  --dataset DATASET  Dataset/ROS Bag name
  --debug            display debug messages
  --mode MODE        Read Radar raw from "CAN" bus or "ROS" topic
  --skip SKIP        skip seconds
  --topic TOPIC      Read Radar raw from a ROS topic
  --visual           Visualize the Radar tracks


rosrun kvaser esr_listener.py --mode=ROS --dataset=/home/kchen/data/radar/radar_2016-10-12-15-59-24-walking.bag --visual
```
