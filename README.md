# Udacity-SDC-Radar-Driver-Micro-Challenge
Udacity Self-Driving Car Radar Driver Micro Challenge

## Purpose:
The Lincoln MKZ is currently getting a Radar overhaul. While it came with some stock sensors, we are adding an additional six radar emitters to the Udacity car in order to not only implement adaptive cruise control features, but to always have access to our surrounding environment and make lane changes and safe stops possible.

Unfortunately, our sensors aren’t supported by ROS, the messaging and control architecture we are using for the whole system. For this first micro-challenge, we need help getting our computers recognizing the tracking data that these systems output.

## Details:
Udacity has purchased six Delphi radar emitters/receivers. The two Delphi ESR 2.5 models will be centered in the front and back of the vehicle, while the Delphi SRR2 units will be paired and side facing on the front and back ends of the vehicle.

Build Environment: ROS Indigo and Ubuntu 14.04
Hardware: 
(2) Delphi ESR 2.5 (24VDC) L2C0051TR for front/rear direct
(4) Delphi SRR2 Electronically Scanning Radars for front/rear paired side view
These units are paired, so it’s really just two sets, and each set has an emitter on each side.

We will be communicating with the Radar units over the CAN bus using Kvaser Leaf Light v2.0 USB converters.

## Additional Information for the ai-world-car-team-c2 team
Read the full challenge in the link to Challenge Google Docs below.  Anyone is welcome to fork and branch.  Please PM diyjac in the nd013.slack.com ai-world-car-team-c2 to submit pull requests.

### Do not use PolySync.
Need to communicate directly with the Kvaser Leaf Light v2.0 USB interface.

### Build Environment: ROS Indigo and Ubuntu 14.04

## Documents:
  1. Challenge Google Docs:
   * https://docs.google.com/document/d/1a2NMVwxTx6m2rpDW3x3E5-KDsuuLbEmlSGozro_bKdA/edit
  2. Link to Kvaser Leaf Light v2 Docs:
   * https://www.kvaser.com/product/kvaser-leaf-light-hs-v2/
  3. Link to Delphi ESR and SRR2 (Short Range Radar-2).
   * Delphi ESR link:  http://www.delphi.com/manufacturers/auto/safety/active/electronically-scanning-radar
   * ESR Datasheet:    http://www.autonomoustuff.com/wp-content/uploads/2016/08/delphi-esr.pdf
   * Delphi SRR2 link: http://www.delphi.com/manufacturers/auto/safety/active/sds
   * SRR2 Datasheet:   http://www.autonomoustuff.com/wp-content/uploads/2016/08/delphi-srr2.pdf
  4. Link to CANopen driver framework for ROS Indigo (experimental - LGPL-3.0) - Interface Candidate 1.
   * Overview:         http://wiki.ros.org/ros_canopen
   * Github link:      https://github.com/ros-industrial/ros_canopen
   * Issues:           https://github.com/ros-industrial/ros_canopen/issues
   * ROS Index:        http://rosindex.github.io/p/canopen_chain_node/github-ros-industrial-ros_canopen/
  5. Link to ROS Device Driver (Roll our own) - Interface Candidate 2.
   * ROS Tutorial:     http://wiki.ros.org/ROS/Tutorials/Creating%20a%20Simple%20Hardware%20Driver
   * ROS Sensors:      http://wiki.ros.org/Sensors
   * ROS Node/Driver:  http://answers.ros.org/question/202000/ros-node-ros-driver-and-ros-wrapper/
   * ROS Driver Video: https://www.youtube.com/watch?v=pagC2WXT1x0
