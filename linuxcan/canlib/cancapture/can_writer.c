/*
**                Copyright 2012 by Kvaser AB, Mölndal, Sweden
**                        http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ===============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ===============================================================================
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** ---------------------------------------------------------------------------
**/

/*
 * Kvaser Linux Canlib
 * Write CAN messages and print out their status back to ros can_raw channel
 */

#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include "vcanevt.h"
// #include <ros/ros.h>
// #include "kvaser/CANPacket.h"

#define canOPEN_NO_INIT_ACCESS 0x0100
#define canOPEN_OVERRIDE_EXCLUSIVE 0x0040

int i = 0;
unsigned char willExit = 0;
int last;
time_t last_time = 0;


void sighand (int sig)
{
  switch (sig) {
  case SIGINT:
    willExit = 1;
    alarm(0);
    break;
  }
}

int Check(char* id, canStatus stat)
{
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
    return(-1);
  }
  return 0;
}
 
int main (int argc, char *argv[])

{
  unsigned long tmp;
  canHandle h;
  canStatus stat;
  int ret = -1;
  int fd = 0;
  unsigned short canId; 
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  // unsigned long t;
  int channel = 0;
  int bitrate = BAUD_500K;
  int j;
  char buf[50];
  // kvaser::CANPacket candat;

  // ros::init(argc, argv, "can_writer");
  // ros::NodeHandle n;
 
  // ros::Publisher can_pub = n.advertise<kvaser::CANPacket>("rosout", 10);

  errno = 0;
  if (argc < 5 || (channel = atoi(argv[1]), errno) != 0 ||
     (canId = strtol(argv[2], NULL, 16), errno) != 0 ||
     (flag = atoi(argv[3]), errno) != 0 ||
     (dlc = atoi(argv[4]), errno) != 0 ||
     (argc != (int)(dlc+5)) ||
     (dlc>0 && (msg[0] = strtol(argv[5], NULL, 16), errno) != 0) ||
     (dlc>1 && (msg[1] = strtol(argv[6], NULL, 16), errno) != 0) ||
     (dlc>2 && (msg[2] = strtol(argv[7], NULL, 16), errno) != 0) ||
     (dlc>3 && (msg[3] = strtol(argv[8], NULL, 16), errno) != 0) ||
     (dlc>4 && (msg[4] = strtol(argv[9], NULL, 16), errno) != 0) ||
     (dlc>5 && (msg[5] = strtol(argv[10], NULL, 16), errno) != 0) ||
     (dlc>6 && (msg[6] = strtol(argv[11], NULL, 16), errno) != 0) ||
     (dlc>7 && (msg[7] = strtol(argv[12], NULL, 16), errno) != 0)) {
    printf("usage %s channel canID flag msgSize(max=8) [msgByte1] [msgByte2]...[msgByte8]\n", argv[0]);
    exit(1);
  }

  /* Use sighand as our signal handler */
  signal(SIGALRM, sighand);
  signal(SIGINT, sighand);
  alarm(1);

  /* Allow signals to interrupt syscalls(in canReadBlock) */
  siginterrupt(SIGINT, 1);
  
  /* Display channel CAP */
  stat = canGetChannelData(channel, canCHANNELDATA_CHANNEL_CAP, &tmp, sizeof(tmp));
  if (Check("canGetChannelData canCHANNELDATA_CHANNEL_CAP", stat)) {
    return ret;
  } 
   
  printf("Channel Capabilities =  0x%08lx ", tmp);
  if (tmp & canCHANNEL_CAP_EXTENDED_CAN) printf("Ext ");
  if (tmp & canCHANNEL_CAP_BUS_STATISTICS) printf("Stat ");
  if (tmp & canCHANNEL_CAP_ERROR_COUNTERS) printf("ErrCnt ");
  if (tmp & canCHANNEL_CAP_CAN_DIAGNOSTICS) printf("Diag ");
  if (tmp & canCHANNEL_CAP_GENERATE_ERROR) printf("ErrGen ");
  if (tmp & canCHANNEL_CAP_GENERATE_OVERLOAD) printf("OvlGen ");
  if (tmp & canCHANNEL_CAP_TXREQUEST) printf("TxRq ");
  if (tmp & canCHANNEL_CAP_TXACKNOWLEDGE) printf("TxAck ");
  if (tmp & canCHANNEL_CAP_VIRTUAL) printf("Virt ");
  if (tmp & canCHANNEL_CAP_SIMULATED) printf("Simulated ");
  if (tmp & canCHANNEL_CAP_REMOTE) printf("Remote ");
  if (tmp & canCHANNEL_CAP_CAN_FD) printf("Fd ");
  if (tmp & canCHANNEL_CAP_CAN_FD_NONISO) printf("Non-ISO ");
  if (tmp & canCHANNEL_CAP_SILENT_MODE) printf("Silent ");
  if (tmp & canCHANNEL_CAP_SINGLE_SHOT) printf("Single shot ");    
  printf("\n");

  stat = canGetChannelData(channel, canCHANNELDATA_TRANS_CAP, &tmp, sizeof(tmp));
  if (!Check("canGetChannelData canCHANNELDATA_TRANS_CAP", stat)) {
    printf("DRVcan Capabilities =   0x%08lx ", tmp);
    if (tmp & canDRIVER_CAP_HIGHSPEED) printf("HiSpd ");
    printf("\n");
  }

  stat = canGetChannelData(channel, canCHANNELDATA_CHANNEL_FLAGS, &tmp, sizeof(tmp));
  if (!Check("canGetChannelData canCHANNELDATA_CHANNEL_FLAGS", stat)) {
    printf("Channel Flags =         0x%08lx ", tmp);
    if (tmp & canCHANNEL_IS_OPEN)  printf("Open ");
    if (tmp & canCHANNEL_IS_CANFD) printf("as canFD ");
    fd = -1;
    printf("\n");
  }

  /* Open channels, parameters and go on bus */
  if (fd) {
    printf("canOpenChannel %d with canOPEN_NO_INIT_ACCESS\n", channel);
    h = canOpenChannel(channel, canOPEN_NO_INIT_ACCESS | canOPEN_OVERRIDE_EXCLUSIVE | canOPEN_ACCEPT_VIRTUAL | canOPEN_CAN_FD);
  } else {
    printf("canOpenChannel %d with canOPEN_EXCLUSIVE\n", channel);
    h = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
  }
  if (h < 0) {
    printf("canOpenChannel %d failed\n", channel);
    return ret;
  }
    
  canSetBusParams(h, bitrate, 4, 3, 1, 1, 0);
  canSetBusOutputControl(h, canDRIVER_NORMAL);
  canBusOn(h);

  printf("Sending canID 0x%x size %d message: '", canId, dlc);
  if (dlc>0) {
    printf("0x%x", msg[0]);
  }
  for (j=1; j<(int)dlc; j++) {
    printf(" 0x%x", msg[j]);
  }
  printf("' with flag=%d to channel %d\n", flag, channel);

  stat = canWrite(h, canId, msg, dlc, flag);
  if (Check("canWrite", stat)) {
     return ret;
  }

  stat = canWriteSync(h, canId);
  if (Check("canWriteSync", stat)) {
     return ret;
  }
   
  canClose(h);
  ret = 0;
  sighand(SIGALRM);
  buf[0] = '\0';
  canGetErrorText(stat, buf, sizeof(buf));
  printf("0x%x: successful, stat=%d (%s)\n", canId, (int)stat, buf);

  return ret;
}


