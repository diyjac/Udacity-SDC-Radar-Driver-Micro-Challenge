from ctypes import *

import sys
import canlib.canlib as canlib

def setUpChannel(channel=0,
                 openFlags=canlib.canOPEN_ACCEPT_VIRTUAL,
                 bitrate=canlib.canBITRATE_500K,
                 bitrateFlags=canlib.canDRIVER_NORMAL):
    cl = canlib.canlib()
    ch = cl.openChannel(channel, openFlags)
    print("Using channel: %s, EAN: %s" % (ch.getChannelData_Name(),
                                          ch.getChannelData_EAN()))
    ch.setBusOutputControl(bitrateFlags)
    ch.setBusParams(bitrate)
    ch.busOn()
    return ch


def tearDownChannel(ch):
    ch.busOff()
    ch.close()

cl = canlib.canlib()
print("canlib version: %s" % cl.getVersion())


channel_0 = 0
channel_1 = 1

ch1 = setUpChannel(channel=1)

if(len(sys.argv) == 1):
    print("Usage python canESRplay.py ESRDatafile ESRDatafile");

class ESR(Structure):
    _fields_ = ("stat", c_int),("id",c_long),("time",c_long),("flag",c_uint),("dlc",c_uint),("msg",c_char * 8)




mystruct = ESR()


for filename in sys.argv[1:]:

    print("Opening file %s",filename);
    f = open(filename, "rb")

    while (f.readinto(mystruct) != 0):
        print mystruct.stat
        print mystruct.id
        print mystruct.msg
        
        msgId = mystruct.stat
        msg = mystruct.msg
        flg = canlib.canMSG_EXT
        ch1.write(msgId, mystruct.msg, flg)
    f.close()


tearDownChannel(ch1)
