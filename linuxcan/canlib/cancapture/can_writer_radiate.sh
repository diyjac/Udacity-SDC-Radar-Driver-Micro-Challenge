#! /bin/bash
## Activate the ESR radar through can bus
## ref: http://www.kvaser.com/canlib-webhelp/page_user_guide.html
# usage: ./can_writer channel canID flag msgSize(max=8) [msgByte1] [msgByte2]...[msgByte8]

# ./can_writer 0 0X4F1 0 8 0 0 0 0 0 0 0XBF 0
# need to port changes in simplewrite.c to can_writer and test before useing can_writer

./simplewrite 0 

###
### for development and testing on the hardware. simplewrite.c is used:
### ./simplewrite 0
###
