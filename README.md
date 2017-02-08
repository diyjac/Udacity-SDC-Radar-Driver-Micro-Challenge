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

### Initial Testing
Use rosbag to see json of track records

`./view_rosbag_radar.py --dataset ~/Downloads/radar_2016-10-12-15-59-24-walking.bag | python -mjson.tool | less`

### Current Development Candidate 1

`./ros/src/sensing/drivers/can/packages/kvaser/nodes/esr_listener/esr_listener.py`

### Issues
  *  Need to develop ROS message to hold ESR Json data.
     1.  ESR data is hierarchical, but the current parser flattens the data.
     2.  Autoware has message compilation for C++, but not for Python
     3.  May need to port esr_listener.py to C++, and produce a hierarchical parser.

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
| Label | Size | Type | Description
| :---: | :---: | :---: | :---: |
| stat  |     4 | enum | Enumerated status - see Stat Enum table below |
| id    |     8 | long | 8 byte message Id |
| time  |     8 | epoch | CAN syncTime |
| flag  |     4 | unsigned int | CAN syncFlag |
| dlc   |     4 | unsigned int | bytes in message (<8) |
| msg   |     8 | char[8] | message |

### Stat Enum
| Label | Value | Description | Troubleshoot |
| :---: | :---: | :---: | :---: |
| canOK |     0 | Normal successful completion | The driver is just fine, and really believes it carried out your command to everyone's satisfaction. |
| canERR_PARAM | -1 | Error in one or more parameters | a parameter ref canERR_xxx specified in the call was invalid, out of range, or so. This status code will also be returned when the call is not implemented. |
| canERR_NOMSG | -2 | There were no messages to read | A function tried to read a message, but there was no message to read. |
| canERR_NOTFOUND | -3 | Specified device or channel not found. | There is no hardware available that matches the given search criteria. For example, you may have specified \ref canOPEN_REQUIRE_EXTENDED but there's no controller capable of extended CAN. You may have specified a channel number that is out of the range for the hardware in question. You may have requested exclusive access to a channel, but the channel is already occupied. |
| canERR_NOMEM | -4 | Out of memory | A memory allocation failed. |
| canERR_NOCHANNELS | -5 | No channels available | There is indeed hardware matching the criteria you specified, but there are no channels available, or the channel you specified is already occupied. |
| canERR_INTERRUPTED | -6 | Interrupted by signals | User CTRL C the capture utility |
| canERR_TIMEOUT | -7 | Timeout occurred | A function waited for something to happen (for example, the arrival of a message), but that something didn't happen. |
| canERR_NOTINITIALIZED | -8 | The library is not initialized | The driver is not initialized. canInitializeLibrary() was probably not called? |
| canERR_NOHANDLES | -9 | Out of handles | No handles are available inside canlib32. The application has too many handles open (i.e. has called \ref canOpenChannel() too many times, or there's a memory leak somewhere.) note: We are not talking about Windows handles here, it's CANLIB's own internal handles. |
| canERR_INVHANDLE | -10 | Handle is invalid | The CANLIB handle you specified (if the API call includes a handle) is not valid. Ensure you are passing the handle and not, for example, a channel number. |
| canERR_INIFILE | -11 | Error in the ini-file (16-bit only) |  |
| canERR_DRIVER  | -12 | Driver type not supported | CAN driver mode is not supported by the present hardware. |
| canERR_TXBUFOFL | -13 | Transmit buffer overflow | The transmit queue was full, so the message was dropped. |
| canERR_RESERVED_1 | -14 | Reserved |  |
| canERR_HARDWARE | -15 | A hardware error has occurred | Something probably related to the hardware happened. This could mean that the device does not respond (IRQ or address conflict?), or that the response was invalid or unexpected (faulty card?). |
| canERR_DYNALOAD | -16 | A driver DLL can't be found or loaded | (One of) the DLL(s) specified in the registry failed to load. This could be a driver installation problem. |
| canERR_DYNALIB | -17 | A DLL seems to have wrong version | DLL version mismatch. (One of) the DLL(s) specified in the registry is - probably - too old, or - less likely - too new. |
| canERR_DYNAINIT | -18 | Error when initializing a DLL | Something failed when a device driver was being initialized. In other words, we can open the driver but it makes a lot of fuss about something we don't understand. |
| canERR_NOT_SUPPORTED | -19 | Operation not supported by hardware or firmware |   |
| canERR_RESERVED_5 | -20 | Reserved |  |
| canERR_RESERVED_6 | -21 | Reserved |  |
| canERR_RESERVED_2 | -22 | Reserved |  |
| canERR_DRIVERLOAD | -23 | Can't find or load kernel driver | A device driver (kernel mode driver for NT, VxD for W95/98) failed to load; or the DLL could not open the device. Privileges? Driver file missing? |
| canERR_DRIVERFAILED | -24 | DeviceIOControl failed | Use Win32 GetLastError() to learn what really happened. |
| canERR_NOCONFIGMGR | -25 | Can't find req'd config s/w (e.g. CS/SS) |  |
| canERR_NOCARD | -26 | The card was removed or not inserted |  |
| canERR_RESERVED_7 | -27 | Reserved |  |
| canERR_REGISTRY | -28 | Error (missing data) in the Registry | A registry key is missing, invalid, malformed, has gone for lunch or what not. can_verify.exe might provide some insight. |
| canERR_LICENSE | -29 | The license is not valid. |  |
| canERR_INTERNAL | -30 | Internal error in the driver | Indicates an error condition in the driver or DLL, which couldn't be properly handled. Please contact the friendly support at support@kvaser.com. |
| canERR_NO_ACCESS | -31 | Access denied | This means that you tried to set the bit rate on a handle to which you haven't got init access or you tried to open a channel that already is open with init access. See \ref canOpenChannel() for more information about init access. |
| canERR_NOT_IMPLEMENTED | -32 | Not implemented | The requested feature or function is not implemented in the device you are trying to use it on. |
| canERR_DEVICE_FILE | -33 | Device File error | An error has occured when trying to access a file on the device. |
| canERR_HOST_FILE | -34 | Host File error | An error has occured when trying to access a file on the host. |
| canERR_DISK | -35 | Disk error | A disk error has occurred. Verify that the disk is initialized. |
| canERR_CRC | -36 | CRC error | The CRC calculation did not match the expected result. |
| canERR_CONFIG | -37 | Configuration Error | The configuration is corrupt. |
| canERR_MEMO_FAIL | -38 | Memo Error | Other configuration error. |
| canERR_SCRIPT_FAIL | -39 | Script Fail | A script has failed.  Note: This code represents several different failures, for example: - Trying to load a corrupt file or not a .txe file.  - Trying to start a t script that has not been loaded.  - Trying to load a t script compiled with the wrong version of the t compiler.  - Trying to unload a t script that has not been stopped.  - Trying to use an envvar that does not exist. |
| canERR_SCRIPT_WRONG_VERSION | -40 | The t script version dosen't match the version(s) that the device firmware supports. |  |
| canERR__RESERVED | -41 | Reserved |  |

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

## Contributing

All help is welcome! Udacity-SDC-Radar-Driver-Micro-Challenge project is an ongoing and live project.  If you'd like to add a new ROS node, open an issue and even submit it yourself.  If you're new to open source or to ROS, I'll help you get your first commit in, ping me.

## License

Udacity-SDC-Radar-Driver-Micro-Challenge is released under [MIT License](./LICENSE)

