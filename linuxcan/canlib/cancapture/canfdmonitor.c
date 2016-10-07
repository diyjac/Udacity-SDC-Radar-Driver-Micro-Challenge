/*
**             Copyright 2012-2016 by Kvaser AB, Molndal, Sweden
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
 * Read CAN FD messages and print out their contents
 */

#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>


#define ALARM_INTERVAL_IN_S   (1)
#define READ_WAIT_INFINITE    (unsigned long)(-1)


static unsigned int msgCounter;


static void check(char* id, canStatus stat)
{
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}

static void printUsageAndExit(char *prgName)
{
  printf("Usage: '%s <channel>'\n", prgName);
  exit(1);
}

static void sighand(int sig)
{
  static unsigned int last;

  switch (sig) {
  case SIGINT:
    break;
  case SIGALRM:
    if (msgCounter != last) {
      printf("rx : %d total: %d\n", msgCounter - last, msgCounter);
    }
    last = msgCounter;
    alarm(ALARM_INTERVAL_IN_S);
    break;
  }
}


int main(int argc, char *argv[])
{
  canHandle hnd;
  canStatus stat;
  int channel;

  if (argc != 2) {
    printUsageAndExit(argv[0]);
  }

  {
    char *endPtr = NULL;
    errno = 0;
    channel = strtol(argv[1], &endPtr, 10);
    if ( (errno != 0) || ((channel == 0) && (endPtr == argv[1])) ) {
      printUsageAndExit(argv[0]);
    }
  }

  printf("Reading messages on channel %d\n", channel);

  /* Use sighand as our signal handler */
  signal(SIGALRM, sighand);
  signal(SIGINT, sighand);

  /* Allow signals to interrupt syscalls */
  siginterrupt(SIGINT, 1);

  /* Open channels, parameters and go on bus */
  hnd = canOpenChannel(channel, canOPEN_CAN_FD);
  if (hnd < 0) {
    printf("canOpenChannel %d", channel);
    check("", hnd);
    return -1;
  }

  stat = canSetBusParams(hnd, canFD_BITRATE_500K_80P, 0, 0, 0, 0, 0);
  check("canSetBusParams", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canSetBusParamsFd(hnd, canFD_BITRATE_500K_80P, 0, 0, 0);
  check("canSetBusParamsFd", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canBusOn(hnd);
  check("canBusOn", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }

  alarm(ALARM_INTERVAL_IN_S);

  do {
    long id;
    unsigned char msg[64];
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;

    stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);

    if (stat == canOK) {
      char *can_std;
      unsigned int i;

      msgCounter++;
      if (flag & canMSG_ERROR_FRAME) {
        printf("(%u) ERROR FRAME flags:0x%x time:%lu\n", msgCounter, flag, time);
        continue;
      }

      if (flag & canFDMSG_FDF) {
        if (flag & canFDMSG_BRS) {
          can_std = "FD+";
          }
        else {
          can_std = "FD ";
        }
      }
      else {
        can_std = "STD";
      }

      printf("CH:%2d %s:%s:%2u:%08lx", channel,
             can_std,
             (flag & canMSG_EXT) ? "X" : " ",
             dlc,id);

      if (flag & canFDMSG_ESI) {
        printf("ESI ");
      }

      printf(" flags:0x%x time:%lu", flag, time);

      for (i = 0; i < dlc; i++) {
        unsigned char byte = msg[i];

        if ((i % 16) == 0) {
          printf("\n    ");
        }
        printf(" %02x ", byte);
      }
      printf("\n");
    } 
    else {
      if (errno == 0) {
        check("\ncanReadWait", stat);
      }
      else {
        perror("\ncanReadWait error");
      }
    }

  } while (stat == canOK);

  sighand(SIGALRM);

ErrorExit:

  alarm(0);
  stat = canBusOff(hnd);
  check("canBusOff", stat);
  stat = canClose(hnd);
  check("canClose", stat);

  return 0;
}
