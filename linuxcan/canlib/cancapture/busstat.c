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

#include <canlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vcanevt.h>
#include <errno.h>

static const long bitrate = canBITRATE_500K;

void check(char* id, canStatus stat)
{

  if (stat != canOK) {
    char buf[50];

    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }

  return;
}

static char* busStatToStr(const unsigned long flag) {
    char* tempStr = NULL;
    #define MACRO2STR(x) case x: tempStr = #x; break
    switch (flag) {
        MACRO2STR( CHIPSTAT_BUSOFF        );
        MACRO2STR( CHIPSTAT_ERROR_PASSIVE );
        MACRO2STR( CHIPSTAT_ERROR_WARNING );
        MACRO2STR( CHIPSTAT_ERROR_ACTIVE  );
        default: tempStr = ""; break;
    }
    #undef MACRO2STR
    return tempStr;
}

void callback (canNotifyData *nd)
{
  switch (nd -> eventType) {
  case canEVENT_STATUS:
    printf("canEVENT_STATUS\n");
    printf("busStatus: %s\n", busStatToStr(nd -> info.status.busStatus));
    printf("TXerror  : %d\n", nd -> info.status.txErrorCounter);
    printf("RXerror  : %d\n", nd -> info.status.txErrorCounter);
    printf("Time     : %ld\n", nd -> info.status.time);
    break;
  case canEVENT_ERROR:
    printf("canEVENT_ERROR\n");
    break;
  case canEVENT_TX:
    printf("canEVENT_TX\n");
    break;
  case canEVENT_RX:
    printf("canEVENT_RX\n");
    break;
  }
  return;
}

void incBusLoad (int channel, int flags, int load)
{
  canHandle handle;
  unsigned char msg[8] = "Kvaser!";
  int i, id = 100, dlc = sizeof(msg);
  canStatus stat = canERR_PARAM;
  handle  = canOpenChannel(channel, flags);
  if (handle < 0) {
    check("canOpenChannel", handle);
    return;
  }
  stat = canBusOff(handle);
  check("canBusOff", stat);
  stat = canSetBusParams(handle, bitrate, 0, 0, 0, 0, 0);
  check("canSetBusParams", stat);
  canBusOn(handle);
  for (i = 0; i < load; i++) {
    stat = canWrite(handle, id, &msg, dlc, 0);
    check("canWrite", stat);
  }
  stat = canWriteWait(handle, id, &msg, dlc, 0, -1);
  check("canWriteWait", stat);
  canBusOff(handle);
  canClose(handle);
}

void printBusStatistics(canBusStatistics *pbstat)
{
  if (!pbstat) return;
  printf("-----------------\n");
  printf("stdData = %lu\n", pbstat->stdData);
  printf("stdRemote = %lu\n", pbstat->stdRemote);
  printf("extData = %lu\n", pbstat->extData);
  printf("extRemote = %lu\n", pbstat->extRemote);
  printf("errFrame = %lu\n", pbstat->errFrame);
  printf("busLoad = %lu\n", pbstat->busLoad);
  printf("overruns = %lu\n", pbstat->overruns);
  printf("-----------------\n");
}

void testBusLoad(canHandle handle)
{
  canStatus stat;
  canBusStatistics bstat;
  stat = canRequestBusStatistics(handle);
  check("canRequestBusStatistics", stat);
  usleep(500000);
  stat = canGetBusStatistics(handle, &bstat, sizeof(bstat));
  check("canGetBusStatistics", stat);
  printBusStatistics(&bstat);
}

static void printUsageAndExit(char *prgName)
{
  printf("Usage: '%s <channel rx> <channel tx>'\n", prgName);
  exit(1);
}


int main(int argc, char *argv[])
{
  canStatus stat;
  canHandle hnd;
  int channelRx;
  int channelTx;

  if (argc != 3) {
    printUsageAndExit(argv[0]);
  }

  {
    char *endPtr = NULL;
    errno = 0;
    channelRx = strtol(argv[1], &endPtr, 10);
    if ( (errno != 0) || ((channelRx == 0) && (endPtr == argv[1])) ) {
      printUsageAndExit(argv[0]);
    }
    channelTx = strtol(argv[2], &endPtr, 10);
    if ( (errno != 0) || ((channelTx == 0) && (endPtr == argv[2])) ) {
      printUsageAndExit(argv[0]);
    }
  }

  hnd  = canOpenChannel(channelRx, canOPEN_REQUIRE_EXTENDED);
  if (hnd < 0) {
    printf("canOpenChannel %d", channelRx);
    check("", hnd);
    return -1;
  }

  stat = canBusOff(hnd);
  check("canBusOff", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canSetNotify(hnd, callback, canNOTIFY_ERROR | canNOTIFY_STATUS, NULL);
  check("canSetNotify", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canSetBusParams(hnd, bitrate, 0, 0, 0, 0, 0);
  check("canSetBusParams", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canBusOn(hnd);
  check("canBusOn", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }

  incBusLoad(channelTx, canOPEN_REQUIRE_EXTENDED, 0);
  testBusLoad(hnd);
  incBusLoad(channelTx, canOPEN_REQUIRE_EXTENDED, 100);
  testBusLoad(hnd);
  testBusLoad(hnd);
  incBusLoad(channelTx, canOPEN_REQUIRE_EXTENDED, 300);
  testBusLoad(hnd);
  testBusLoad(hnd);

ErrorExit:

  stat = canBusOff(hnd);
  check("canBusOff", stat);
  stat = canClose(hnd);
  check("canClose", stat);

  return 0;
}

