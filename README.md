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
  1.  Install Ubuntu 14.04:              http://releases.ubuntu.com/14.04/
  2.  Install ROS Indigo:                http://wiki.ros.org/indigo/Installation/Ubuntu
  3.  Clone this repository:             git clone https://github.com/diyjac/Udacity-SDC-Radar-Driver-Micro-Challenge.git
  4.  Download Kvaser linuxcan:          http://www.kvaser.com/software/7330130980754/V5_18_0/linuxcan.tar.gz
  5.  Change directory:                  cd Udacity-SDC-Radar-Driver-Micro-Challenge/
  6.  Untar linuxcan.tar.gz:             tar zxvf ~/Download/linuxcan.tar.gz
  7.  Change directory:                  cd linuxcan
  8.  Make install:                      sudo make install
  9.  Copy Sample data to /tmp:          cp ../sampleData/rawESR6.data /tmp/rawESR.data 
  10. Install vcanplayback (alpha):      cd vcanplayback; make install; sudo ./installscript.sh
  11. Start vcanplayback kernel module:  sudo ./vcanplayback.sh start
  12. Make cancapture (alpha):           cd ../canlib/cancapture; make
  13. Verify (alpha):                    ./listChannels
   *  Found 1 channel(s).
   *  channel  0 = Kvaser Virtual CAN,	0-00000-00000-0, 0, 0.0.0.0 

### Raw ESR data format (fixed length records)
| Info  | stat  | id    | time  | flag  | dlc   | msg   |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Size  |     4 |     8 |     8 |     4 |     4 |     8 |
| Type  | enum  | long  | epoch | int   | int   | char[8] |

## Documents:
  1. Challenge Google Docs:
   * https://docs.google.com/document/d/1a2NMVwxTx6m2rpDW3x3E5-KDsuuLbEmlSGozro_bKdA/edit
  2. Link to Kvaser Leaf Light v2 Docs:
   * https://www.kvaser.com/product/kvaser-leaf-light-hs-v2/
  3. Link to Delphi ESR and SRR2 (Short Range Radar-2).
   * Delphi ESR link:   http://www.delphi.com/manufacturers/auto/safety/active/electronically-scanning-radar
   * ESR Datasheet:     http://www.autonomoustuff.com/wp-content/uploads/2016/08/delphi-esr.pdf
   * Delphi SRR2 link:  http://www.delphi.com/manufacturers/auto/safety/active/sds
   * SRR2 Datasheet:    http://www.autonomoustuff.com/wp-content/uploads/2016/08/delphi-srr2.pdf
  4. Link to ROS Device Driver (Roll our own) - Interface Candidate 1.
   * ROS Tutorial:      http://wiki.ros.org/ROS/Tutorials/Creating%20a%20Simple%20Hardware%20Driver
   * ROS Sensors:       http://wiki.ros.org/Sensors
   * ROS Node/Driver:   http://answers.ros.org/question/202000/ros-node-ros-driver-and-ros-wrapper/
   * ROS Driver Video:  https://www.youtube.com/watch?v=pagC2WXT1x0
   * Kvaser linuxcan:   http://www.kvaser.com/software/7330130980754/V5_18_0/linuxcan.tar.gz

