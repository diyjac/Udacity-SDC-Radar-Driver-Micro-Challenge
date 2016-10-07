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


void check (char* id, canStatus stat)
{
  char buf[50];

  buf[0] = '\0';
  canGetErrorText(stat, buf, sizeof(buf));
  if (stat != canOK) {
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  } else {
    printf("%s: OK\n", id);
  }
}


/* Set bus parameters and read them back */

int main(int argc, char *argv[])

{
  long freq;
  unsigned int tseg1, tseg2, sjw, noSamp, syncmode;
  canStatus stat = -1;
  int j;
  canHandle hnd[2];

  (void)argc; // Unused.
  (void)argv; // Unused.

  for(j = 0 ; j < 2 ; j++) {
    hnd[j] = canOpenChannel(j, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED);
    if (hnd[j] < 0) {
      printf("canOpenChannel (%d) failed\n", j);
      return -1;
    }
    stat = canSetBusParams(hnd[j], 500000, 4, 3, 1, 1, 0);
    check("canSetBusParams", stat);
  }

  printf("\n");
  for (j = 0; j < 2; j++){
    stat = canGetBusParams(hnd[j], &freq, &tseg1, &tseg2, &sjw, &noSamp, &syncmode);
    printf("hnd[%d]:freq %ld, tseg1 %u, tseg2 %u, sjw %u, noSamp %u, syncmode %u\n ",
           j, freq, tseg1, tseg2, sjw, noSamp, syncmode);
    check("canGetBusParams", stat);
  }

  return 0;
}
