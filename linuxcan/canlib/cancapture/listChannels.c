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
 * List available channels
 */

#include "canlib.h"
#include <stdio.h>
#include <string.h>

 /*
 Lists available CAN channels
 */


int main(int argc, char* argv[])
{
  int chanCount = 0;
  canStatus stat;
  int i;
  char name[256];
  char custChanName[40];
  unsigned int ean[2], fw[2], serial[2];

  (void)argc; // Unused.
  (void)argv; // Unused.

  memset(name, 0, sizeof(name));
  memset(custChanName, 0, sizeof(custChanName));
  memset(ean, 0, sizeof(ean));
  memset(fw, 0, sizeof(fw));
  memset(serial, 0, sizeof(serial));

  stat = canGetNumberOfChannels(&chanCount);
  if (stat != canOK) {
    printf("Error in canGetNumberOfChannels\n");
    exit(1);
  }
  printf("Found %d channel(s).\n", chanCount);

  for (i = 0; i < chanCount; i++) {

    stat = canGetChannelData(i, canCHANNELDATA_DEVDESCR_ASCII,
                             &name, sizeof(name));
    if (stat != canOK) {
      printf("Error in canGetChannelData - DEVDESCR_ASCII\n");
      exit(1);
    }

    if (strcmp(name, "Kvaser Unknown") == 0) {
      stat = canGetChannelData(i, canCHANNELDATA_CHANNEL_NAME,
                               &name, sizeof(name));
      if (stat != canOK) {
        printf("Error in canGetChannelData - CHANNEL_NAME\n");
        exit(1);
      }
    }

    stat = canGetChannelData(i, canCHANNELDATA_CARD_UPC_NO,
                             &ean, sizeof(ean));
    if (stat != canOK) {
      printf("Error in canGetChannelData - CARD_UPC_NO\n");
      exit(1);
    }

    stat = canGetChannelData(i, canCHANNELDATA_CARD_SERIAL_NO,
                             &serial, sizeof(serial));
    if (stat != canOK) {
      printf("Error in canGetChannelData - CARD_SERIAL_NO\n");
      exit(1);
    }

    stat = canGetChannelData(i, canCHANNELDATA_CARD_FIRMWARE_REV,
                             &fw, sizeof(fw));
    if (stat != canOK) {
      printf("Error in canGetChannelData - CARD_FIRMWARE_REV\n");
      exit(1);
    }

    (void) canGetChannelData(i, canCHANNELDATA_CUST_CHANNEL_NAME,
                             custChanName, sizeof(custChanName));

    printf("channel %2.1d = %s,\t%x-%05x-%05x-%x, %u, %u.%u.%u.%u %s\n",
           i, name,
           (ean[1] >> 12), ((ean[1] & 0xfff) << 8) | ((ean[0] >> 24) & 0xff),
           (ean[0] >> 4) & 0xfffff, (ean[0] & 0x0f),
           serial[0],
           fw[1] >> 16, fw[1] & 0xffff, fw[0] >> 16, fw[0] & 0xffff,
           custChanName);
  }

  return 0;
}
