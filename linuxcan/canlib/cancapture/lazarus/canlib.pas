unit CANLIB;

{$mode delphi}

(*
**
**               Copyright 1995-2016 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
**  This software is dual licensed under the following two licenses:
**  BSD-new and GPLv2. You may use either one. See the included
**  COPYING file for details.
**
**  License: BSD-new
**  ===============================================================================
**  Redistribution and use in source and binary forms, with or without
**  modification, are permitted provided that the following conditions are met:
**      * Redistributions of source code must retain the above copyright
**        notice, this list of conditions and the following disclaimer.
**      * Redistributions in binary form must reproduce the above copyright
**        notice, this list of conditions and the following disclaimer in the
**        documentation and/or other materials provided with the distribution.
**      * Neither the name of the <organization> nor the
**        names of its contributors may be used to endorse or promote products
**        derived from this software without specific prior written permission.
**
**  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
**  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
**  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
**  License: GPLv2
**  ===============================================================================
**  This program is free software; you can redistribute it and/or
**  modify it under the terms of the GNU General Public License
**  as published by the Free Software Foundation; either version 2
**  of the License, or (at your option) any later version.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program; if not, write to the Free Software
**  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
*)
(*
** This unit defines an interface for Delphi to libcanlib.so
** It has been tested with Lazarus 1.6.
*)

interface

uses
  Classes, dynlibs;


const

  canINVALID_HANDLE = -1;

  canOK = 0;
  canERR_PARAM = -1; {// Error in parameter}
  canERR_NOMSG = -2; {// No messages available}
  canERR_NOTFOUND = -3; {// Specified hw not found}
  canERR_NOMEM = -4; {// Out of memory}
  canERR_NOCHANNELS = -5; {// No channels avaliable}
  canERR_RESERVED_3 = -6;
  canERR_TIMEOUT = -7; {// Timeout occurred}
  canERR_NOTINITIALIZED = -8; {// Lib not initialized}
  canERR_NOHANDLES = -9; {// Can't get handle}
  canERR_INVHANDLE = -10; {// Handle is invalid}
  canERR_INIFILE = -11; {// Error in the ini-file (16-bit only)}
  canERR_DRIVER = -12; {// CAN driver type not supported}
  canERR_TXBUFOFL = -13; {// Transmit buffer overflow}
  canERR_RESERVED_1 = -14;
  canERR_HARDWARE = -15; {// Some hardware error has occurred}
  canERR_DYNALOAD = -16; {// Can't find requested DLL}
  canERR_DYNALIB = -17; {// DLL seems to be wrong version}
  canERR_DYNAINIT = -18; {// Error when initializing DLL}
  canERR_NOT_SUPPORTED = -19;
  canERR_RESERVED_5 = -20;
  canERR_RESERVED_6 = -21;
  canERR_RESERVED_2 = -22;
  canERR_DRIVERLOAD = -23; {// Can't find/load driver}
  canERR_DRIVERFAILED = -24; {// DeviceIOControl failed; use Win32 GetLastError()}
  canERR_NOCONFIGMGR = -25; {// Can't find req'd config s/w (e.g. CS/SS)}
  canERR_NOCARD = -26; {// The card was removed or not inserted}
  canERR_RESERVED_7 = -27;
  canERR_REGISTRY = -28; // Error in the Registry
  canERR_LICENSE = -29; // The license is not valid.
  canERR_INTERNAL = -30; // Internal error in the driver.
  canERR_NO_ACCESS = -31; // Access denied
  canERR_NOT_IMPLEMENTED = -32; // Requested function is not implemented
  canERR_DEVICE_FILE = -33;
  canERR_HOST_FILE = -34;
  canERR_DISK = -35;
  canERR_CRC = -36;
  canERR_CONFIG = -37;
  canERR_MEMO_FAIL = -38;
  canERR_SCRIPT_FAIL = -39;
  canERR_SCRIPT_WRONG_VERSION = -40;
  canERR__RESERVED = -41; // RESERVED



  //WM__CANLIB = (WM_USER + 16354); {Windows message from Can unit.}

  canEVENT_RX = 32000; {Receive event}
  canEVENT_TX = 32001; {Transmit event}
  canEVENT_ERROR = 32002; {Error event}
  canEVENT_STATUS = 32003; {Change-of-status event}
  canEVENT_ENVVAR = 32004; {An envvar changed}
  canEVENT_BUSONOFF = 32005; {Bus on/off status changed}
  canEVENT_REMOVED = 32006; {Device removed}

   {Used in canSetNotify}
  canNOTIFY_RX = $0001; { Notify on receive }
  canNOTIFY_TX = $0002; { Notify on transmit }
  canNOTIFY_ERROR = $0004; { Notify on error }
  canNOTIFY_STATUS = $0008; { Notify on (some) status change events }
  canNOTIFY_ENVVAR = $0010; { An environment variable was changed by a script }
  canNOTIFY_BUSONOFF = $0020; { Notify on bus on/off status changed }
  canNOTIFY_REMOVED = $0040; { Notify on device removed }


{Circuit status flags}
  canSTAT_ERROR_PASSIVE = $00000001; {The circuit is error passive}
  canSTAT_BUS_OFF = $00000002; {The circuit is Off Bus}
  canSTAT_ERROR_WARNING = $00000004; {At least one error counter > 96}
  canSTAT_ERROR_ACTIVE = $00000008; {The circuit is error active.}
  canSTAT_TX_PENDING = $00000010; {There are messages pending transmission}
  canSTAT_RX_PENDING = $00000020; {There are messages in the receive buffer}
  canSTAT_RESERVED_1 = $00000040;
  canSTAT_TXERR = $00000080; {There has been at least one TX error}
  canSTAT_RXERR = $00000100; {There has been at least one RX error of some sort}
  canSTAT_HW_OVERRUN = $00000200; {The has been at least one HW buffer overflow}
  canSTAT_SW_OVERRUN = $00000400; {The has been at least one SW buffer overflow}


   {Message information flags}
  canMSG_MASK = $00FF; { Used to mask the non-info bits }
  canMSG_RTR = $0001; { Message is a remote request }
  canMSG_STD = $0002; { Message has a standard ID }
  canMSG_EXT = $0004; { Message has an extended id }
  canMSG_WAKEUP = $0008; { Message is a SWC wakeup frame}
  canMSG_STATUS = $0008; { Obsolete - retained for compatibility }
  canMSG_NERR = $0010; { Message sent/received with TJA1054 (etc.) NERR active }
  canMSG_ERROR_FRAME = $0020; { Message is an error frame }
  canMSG_TXACK = $0040; { Message is a TX ACK (msg is really sent) }
  canMSG_TXRQ = $0080; { Message is a TX REQUEST (msg is transfered to the chip)}

   {Message error flags, >= $0100}
  canFDMSG_MASK = $ff0000; { Used to mask the non-info bits }
  canFDMSG_EDL = $010000;  { Obsolete, use canFDMSG_FDF instead}
  canFDMSG_FDF = $010000;  { Indicate if message is an FD message }
  canFDMSG_BRS = $020000;  { Indicate if message should be sent with bit rate switch }
  canFDMSG_ESI = $040000;  { Indicate if the sender of this message is in error passive mode }
   {Message error flags, >= $0100}
  canMSGERR_MASK = $FF00; { Used to mask the non-error bits }
    { $0100 reserved }
  canMSGERR_HW_OVERRUN = $0200; { HW buffer overrun }
  canMSGERR_SW_OVERRUN = $0400; { SW buffer overrun }
  canMSGERR_STUFF = $0800; { Stuff error }
  canMSGERR_FORM = $1000; { Form error }
  canMSGERR_CRC = $2000; { CRC error }
  canMSGERR_BIT0 = $4000; { Sent dom, read rec}
  canMSGERR_BIT1 = $8000; { Sent rec, read dom}

   {Convenience values}
  canMSGERR_OVERRUN = $0600; { Any overrun condition.}
  canMSGERR_BIT = $C000; { Any bit error (note: TWO bits)}
  canMSGERR_BUSERR = $F800; { Any RX error}

  canCIRCUIT_ANY = -1; { Any circuit will do }
  canCARD_ANY = -1; { Any card will do}
  canCHANNEL_ANY = -1; { Any channel will do}



   {Flags for canAccept}
  canFILTER_ACCEPT = 1;
  canFILTER_REJECT = 2;
  canFILTER_SET_CODE_STD = 3;
  canFILTER_SET_MASK_STD = 4;
  canFILTER_SET_CODE_EXT = 5;
  canFILTER_SET_MASK_EXT = 6;

  canFILTER_NULL_MASK = 0;

  canDRIVER_NORMAL = 4;
  canDRIVER_SILENT = 1;
  canDRIVER_SELFRECEPTION = 8;
  canDRIVER_OFF = 0;


  { "shortcut" baud rates; use with canBusParams or canTranslateBaud }
  { canBAUD_xxx is obsolete; use canBITRATE_xxx instead }
  canBAUD_1M = -1;
  canBAUD_500K = -2;
  canBAUD_250K = -3;
  canBAUD_125K = -4;
  canBAUD_100K = -5;
  canBAUD_62K = -6;
  canBAUD_50K = -7;
  canBAUD_83K = -8;

  canBITRATE_1M = -1;
  canBITRATE_500K = -2;
  canBITRATE_250K = -3;
  canBITRATE_125K = -4;
  canBITRATE_100K = -5;
  canBITRATE_62K = -6;
  canBITRATE_50K = -7;
  canBITRATE_83K = -8;
  canBITRATE_10K = -9;

  canFD_BITRATE_500K_80P = -1000;
  canFD_BITRATE_1M_80P   = -1001;
  canFD_BITRATE_2M_80P   = -1002;
  canFD_BITRATE_4M_80P   = -1003;
  canFD_BITRATE_8M_60P   = -1004;

  canIOCTL_PREFER_EXT = 1;
  canIOCTL_PREFER_STD = 2;
   { 3,4 reserved }
  canIOCTL_CLEAR_ERROR_COUNTERS = 5;
  canIOCTL_SET_TIMER_SCALE = 6;
  canIOCTL_SET_TXACK = 7;

  canIOCTL_GET_RX_BUFFER_LEVEL = 8;
  canIOCTL_GET_TX_BUFFER_LEVEL = 9;
  canIOCTL_FLUSH_RX_BUFFER = 10;
  canIOCTL_FLUSH_TX_BUFFER = 11;

  canIOCTL_GET_TIMER_SCALE = 12;
  canIOCTL_SET_TX_REQUESTS = 13;

  canIOCTL_GET_EVENTHANDLE = 14;
  canIOCTL_SET_BYPASS_MODE = 15;
  canIOCTL_SET_WAKEUP = 16;

  canIOCTL_GET_DRIVERHANDLE = 17;
  canIOCTL_MAP_RXQUEUE = 18;
  canIOCTL_GET_WAKEUP = 19;
  canIOCTL_SET_REPORT_ACCESS_ERRORS = 20;
  canIOCTL_GET_REPORT_ACCESS_ERRORS = 21;
  canIOCTL_CONNECT_TO_VIRTUAL_BUS = 22;
  canIOCTL_DISCONNECT_FROM_VIRTUAL_BUS = 23;
  canIOCTL_SET_USER_IOPORT = 24;
  canIOCTL_GET_USER_IOPORT = 25;
  canIOCTL_SET_BUFFER_WRAPAROUND_MODE     = 26;
  canIOCTL_SET_RX_QUEUE_SIZE              = 27;
  canIOCTL_SET_USB_THROTTLE               = 28;
  canIOCTL_GET_USB_THROTTLE               = 29;
  canIOCTL_SET_BUSON_TIME_AUTO_RESET      = 30;
  canIOCTL_GET_TXACK                      = 31;
  canIOCTL_SET_LOCAL_TXECHO               = 32;
  canIOCTL_SET_ERROR_FRAMES_REPORTING     = 33;
  canIOCTL_GET_CHANNEL_QUALITY            = 34;
  canIOCTL_GET_ROUNDTRIP_TIME             = 35;
  canIOCTL_GET_BUS_TYPE                   = 36;
  canIOCTL_GET_DEVNAME_ASCII              = 37;
  canIOCTL_GET_TIME_SINCE_LAST_SEEN       = 38;

  //Type of buffer
  canOBJBUF_TYPE_AUTO_RESPONSE = $01;
  canOBJBUF_TYPE_PERIODIC_TX = $02;

  // The buffer responds to RTRs only, not regular messages.
  canOBJBUF_AUTO_RESPONSE_RTR_ONLY = $01;

  // Check for specific version(s) of CANLIB.
  canVERSION_DONT_ACCEPT_LATER = $01;
  canVERSION_DONT_ACCEPT_BETAS = $02;

  CANID_METAMSG = (-1);
  CANID_WILDCARD = (-2);

  kvENVVAR_TYPE_INT = 1;
  kvENVVAR_TYPE_FLOAT = 2;
  kvENVVAR_TYPE_STRING = 3;

  kvEVENT_TYPE_KEY = 1;

  kvSCRIPT_STOP_NORMAL = 0;
  kvSCRIPT_STOP_FORCED = -9;

  kvDEVICE_MODE_INTERFACE = $00;
  kvDEVICE_MODE_LOGGER = $01;

  canVERSION_CANLIB32_VERSION    = 0;
  canVERSION_CANLIB32_PRODVER    = 1;
  canVERSION_CANLIB32_PRODVER32  = 2;
  canVERSION_CANLIB32_BETA       = 3;

  kvBUSTYPE_NONE        = 0;
  kvBUSTYPE_PCI         = 1;
  kvBUSTYPE_PCMCIA      = 2;
  kvBUSTYPE_USB         = 3;
  kvBUSTYPE_WLAN        = 4;
  kvBUSTYPE_PCI_EXPRESS = 5;
  kvBUSTYPE_ISA         = 6;
  kvBUSTYPE_VIRTUAL     = 7;
  kvBUSTYPE_PC104_PLUS  = 8;
  kvBUSTYPE_LAN         = 9;

  kvBUSTYPE_GROUP_VIRTUAL  = 1; ///< ::kvBUSTYPE_VIRTUAL
  kvBUSTYPE_GROUP_LOCAL    = 2; ///< ::kvBUSTYPE_USB
  kvBUSTYPE_GROUP_REMOTE   = 3; ///< ::kvBUSTYPE_WLAN
  kvBUSTYPE_GROUP_INTERNAL = 4; ///< ::kvBUSTYPE_PCI, ::kvBUSTYPE_PCMCIA, ...
  ///
  ///
  kvSCRIPT_REQUEST_TEXT_UNSUBSCRIBE =   1;
  kvSCRIPT_REQUEST_TEXT_SUBSCRIBE   =   2;
  kvSCRIPT_REQUEST_TEXT_ALL_SLOTS   = 255;

type


  { This one is primarily used by WCANKING }
  TMsgRec = record
       {This record holds information about a CAN message.}
    envelope: Longint; {The CAN envelope.}
    dlc: Integer; {The data length code.}
    flag: Integer; {The flag have information about remote request and
                             #X Return flags}
    case indexType: Integer of
      0: (data: array[0..7] of AnsiChar); {CAN data as char.}
      1: (shData: array[0..7] of ShortInt); {CAN data as shortint.}
      2: (bData: array[0..7] of Byte); {CAN data as byte.}
      3: (iData: array[0..3] of SmallInt); {CAN data as smallint.}
      4: (lData: array[0..1] of LongInt); {CAN data as Longint.}
      6: (wData: array[0..3] of Word); {CAN data as word.}
      7: (tData: string[7]); {CAN data as string[7].}
      8: (fData: array[0..1] of Single); {CAN data as float.}
      9: (rData: Real); {CAN data as real.}
      10: (dData: Double); {CAN data as double.}
      11: (cData: Comp); {CAN data as comp.}
  end;


  { This one is primarily used by WCANKING }
  TMsgObj = class(TObject)
     { A TMsgObj holds a TMsgRec, so it can be used as an object in TStringList.}
  public
      {Public declarations}
    txm: Boolean; {True if CAN message sent, false if received.}
    time: LongInt; {Receive time in milliseconds.}
    count: Integer; {Message number.}
    MsgRec: TMsgRec; {The CAN message.}
  end;


  //canMemoryAllocator = TFarProc; {Memory allocator, if nil malloc is used.}
  //canMemoryDeallocator = TFarProc; {Memory deallocator, if nil free is used.}
  //canAction = TFarProc; {Currently unsupported.}
  BYTEPTR = PAnsiChar; {Byte pointer.}

 {Can hardware descriptor, holds information about CAN card
  and CAN circuit used.}
  canHWDescr = record
    circuitType: integer; { The CAN circuit.}
    cardType: integer;
    channel: integer;
  end;

  { Used in canOpen. Obsolete. }
  //canSWDescr = record
  //  rxBufSize: integer; {Requested receive buffer size [1, 32767].}
  //  txBufSize: integer; {Requested transmit buffer size [0, 32767].}
  //  alloc: canMemoryAllocator; {Memory allocator.}
  //  deAlloc: canMemoryDeallocator; {Memory deallocator.}
  //end;

  //canSWDescrPointer = ^canSWDescr;

  TWMCan = record {Type declaration for windows or dos message}
    Msg: Cardinal;
    case Integer of
      0: (WParam: Cardinal;
        LParam: Longint;
        Result: Longint);

      1: (handle: Cardinal; {CAN handle issuing message.}
        minorMsg: Word; {Message types.}
        status: Word; ); {Status.}
  end;

  canStatus = integer;
  canHandle = integer;
  kvEnvHandle = Int64;

  canBusStatistics = record
    stdData: Int64;
    stdRemote: Int64;
    extData: Int64;
    extRemote: Int64;
    errFrame: Int64; // Error frames
    busLoad: Int64; // 0 .. 10000 meaning 0.00-100.00%
    overruns: Int64;
  end;

  canUserIoPortData = record
    portNo: Cardinal;
    portValue: Cardinal;
  end;

  TCanInterface = class(TObject)
  public
   channel : Integer;
   eanhi, eanlo : Cardinal;
   serial : Cardinal;
   hnd : canHandle;
   name: String;
   Constructor create(canChannel: Integer);overload;

  private
  end;

{------------------------------------------------------------+
| End of type definitions.                                   |
+------------------------------------------------------------}

//function canLocateHardware: canStatus; stdcall;
procedure canInitializeLibrary; stdcall;
function canUnloadLibrary: Integer; stdcall;
procedure SetDllName(s: string);




type
  kvCallback_t = procedure(handle: canHandle; context: Pointer; notifyEvent: Cardinal); stdcall;

  kvStatus = canStatus;

  kvTimeDomain = Cardinal; { Really a pointer to something }

  kvTimeDomainData = packed record
    nMagiSyncGroups: Integer;
    nMagiSyncedMembers: Integer;
    nNonMagiSyncCards: Integer;
    nNonMagiSyncedMembers: Integer;
  end;

var
  //canOpen: function(const hwdescr: canHWDescr; swdescr: Pointer; flags: Cardinal): canHandle; stdcall;
  canClose: function(handle: canHandle): canStatus; stdcall;
  canBusOn: function(handle: canHandle): canStatus; stdcall;
  canBusOff: function(handle: canHandle): canStatus; stdcall;
  canSetBusParams: function(handle: canHandle; freq: Int64; tseg1, tseg2, sjw, noSamp, syncmode: Cardinal): canStatus; stdcall;
  canGetBusParams: function(handle: canHandle; var freq: Int64; var tseg1, tseg2, sjw, noSamp, syncmode: Cardinal): canStatus; stdcall;
  canSetBusParamsFd: function(handle: canHandle; freq: Int64; tseg1, tseg2, sjw: Cardinal): canStatus; stdcall;
  canGetBusParamsFd: function(handle: canHandle; var freq: Int64; var tseg1, tseg2, sjw: Cardinal): canStatus; stdcall;
  canSetBusOutputControl: function(handle: canHandle; drivertype: Cardinal): canStatus; stdcall;
  canGetBusOutputControl: function(handle: canHandle; var drivertype: Cardinal): canStatus; stdcall;
  canAccept: function(handle: canHandle; envelope: Int64; flag: Cardinal): canStatus; stdcall;
  canReadStatus: function(handle: canHandle; var flags: Int64): canStatus; stdcall;
  canReadErrorCounters: function(handle: canHandle; var txErr, rxErr, ovErr: Cardinal): canStatus; stdcall;
  canWrite: function(handle: canHandle; id: Int64; msg: Pointer; dlc: Cardinal; flag: Cardinal): canStatus; stdcall;
  canWriteSync: function(handle: canHandle; timeout: Int64): canStatus; stdcall;
  canRead: function(handle: canHandle; var id: Int64; msg: Pointer; var dlc: Cardinal; var flag: Cardinal; var time: Int64): canStatus; stdcall;
  canReadWait: function(handle: canHandle; var id: Int64; msg: Pointer; var dlc: Cardinal; var flag: Cardinal; var time: Int64; timeout: Int64): canStatus; stdcall;
  canReadSpecific: function(handle: canHandle; id: Int64; msg: Pointer; var dlc: Cardinal; var flag: Cardinal; var time: Int64): canStatus; stdcall;
  canReadSync: function(handle: canHandle; timeout: Int64): canStatus; stdcall;
  canReadSyncSpecific: function(handle: canHandle; id, timeout: Int64): canStatus; stdcall;
  canReadSpecificSkip: function(handle: canHandle; id: Int64; msg: Pointer; var dlc: Cardinal; var flag: Cardinal; var time: Int64): canStatus; stdcall;
  //canInstallAction: function(handle: canHandle; id: Longint; fn: Pointer): canStatus; stdcall;
  //canUninstallAction: function(handle: canHandle; id: Longint): canStatus; stdcall;
  //canInstallOwnBuffer: function(handle: canHandle; id: Longint; len: Cardinal; buf: Pointer): canStatus; stdcall;
  //canSetNotify: function(handle: canHandle; aHWnd: HWND; aNotifyFlags: Cardinal): canStatus; stdcall;
  canTranslateBaud: function(var freq: Int64; var tseg1, tseg2, sjw, noSamp, syncMode: Cardinal): canStatus; stdcall;
  canGetErrorText: function(err: canStatus; buf: PAnsiChar; bufsiz: Cardinal): canStatus; stdcall;
  canGetVersion: function: Word; stdcall;
  canIoCtl: function(handle: canHandle; func: Cardinal; buf: Pointer; buflen: Cardinal): canStatus; stdcall;
  canReadTimer: function(handle: canHandle): Int64; stdcall;
  kvReadTimer: function(handle: canHandle; var time: Cardinal): kvStatus; stdcall;
  kvReadTimer64: function(handle: canHandle; var time: Int64): kvStatus; stdcall;
  canGetNumberOfChannels: function(var channelCount: Integer): canStatus; stdcall;
  canGetChannelData: function(channel, item: Integer; var buffer; bufsize: Int64): canStatus; stdcall;
  canOpenChannel: function(channel: Integer; flags: Integer): canHandle; stdcall;
  canWaitForEvent: function(hnd: canHandle; timeout: Cardinal): canStatus; stdcall;
  canSetBusParamsC200: function(hnd: canHandle; btr0, btr1: byte): canStatus; stdcall;
  canGetVersionEx: function(itemCode: Cardinal): Cardinal; stdcall;
  canSetDriverMode: function(hnd: canHandle; lineMode, resNet: Integer): canStatus; stdcall;
  canGetDriverMode: function(hnd: canHandle; var lineMode: Integer; var resNet: Integer): canStatus; stdcall;
  canParamGetCount: function(): canStatus; stdcall;
  canParamCommitChanges: function(): canStatus; stdcall;
  canParamDeleteEntry: function(index: Integer): canStatus; stdcall;
  canParamCreateNewEntry: function(): canStatus; stdcall;
  canParamSwapEntries: function(index1, index2: Integer): canStatus; stdcall;
  canParamGetName: function(index: Integer; buffer: PAnsiChar; maxlen: Integer): canStatus; stdcall;
  canParamGetChannelNumber: function(index: Integer): canStatus; stdcall;
  canParamGetBusParams: function(index: Integer; var bitrate: Int64; var tseg1: Cardinal; var tseg2: Cardinal; var sjw: Cardinal; var nosamp: Cardinal): canStatus; stdcall;
  canParamSetName: function(index: Integer; buffer:  PAnsiChar): canStatus; stdcall;
  canParamSetChannelNumber: function(index, channel: Integer): canStatus; stdcall;
  canParamSetBusParams: function(index: Integer; bitrate: Int64; tseq1, tseq2, sjw, noSamp: Cardinal): canStatus; stdcall;
  canParamFindByName: function(const Name: PAnsiChar):canStatus; stdcall;
  canObjBufFreeAll: function(handle: canHandle): canStatus; stdcall;
  canObjBufAllocate: function(handle: canHandle; tp: Integer): canStatus; stdcall;
  canObjBufFree: function(handle: canHandle; idx: Integer): canStatus; stdcall;
  canObjBufWrite: function(handle: canHandle; idx, id: Integer; var msg; dlc, flags: cardinal): canstatus; stdcall;
  canObjBufSetFilter: function(handle: canHandle; idx: Integer; code, mask: Cardinal): canStatus; stdcall;
  canObjBufSetFlags: function(handle: canHandle; idx: Integer; flags: Cardinal): canStatus; stdcall;
  canObjBufEnable: function(handle: canHandle; idx: Integer): canStatus; stdcall;
  canObjBufDisable: function(handle: canHandle; idx: Integer): canStatus; stdcall;
  canObjBufSetPeriod: function(handle: canHandle; idx: Integer; period: Cardinal): canStatus; stdcall;
  canObjBufSetMsgCount: function(handle: canHandle; idx: Integer; count: Cardinal): canStatus; stdcall;
  canObjBufSendBurst: function(handle: canHandle; idx: Integer; burstLen: Cardinal): canStatus; stdcall;
  canProbeVersion: function(handle: canHandle; major, minor, oem_id: Integer; flags: Cardinal): Boolean; stdcall;
  canResetBus: function(handle: canHandle): canStatus; stdcall;
  canWriteWait: function(handle: canHandle; id: Int64; var msg; dlc, flag : Cardinal; timeout : Int64): canStatus; stdcall;
  canSetAcceptanceFilter: function(handle: canHandle; code, mask: Cardinal; is_extended: Integer): canStatus; stdcall;
  canFlushReceiveQueue: function(handle: canHandle): canStatus; stdcall;
  canFlushTransmitQueue: function(handle: canHandle): canStatus; stdcall;
  canRequestChipStatus:function(handle: canHandle): canStatus; stdcall;
  canRequestBusStatistics: function(handle: canHandle): canStatus; stdcall;
  canGetBusStatistics: function(handle: canHandle; var stat: canBusStatistics; bufsiz: Int64): canStatus; stdcall;
  kvAnnounceIdentity: function(handle: canHandle; var buf; bufsiz: Int64): canStatus; stdcall;
  kvAnnounceIdentityEx: function(handle: canHandle; typ: Integer; var buf; bufsiz: Int64): canStatus; stdcall;
  kvSetNotifyCallback: function(handle: canHandle; callback: kvCallback_t; context: Pointer; notifyFlags: Cardinal): canStatus; stdcall;
  kvBeep: function(handle: canHandle; freq: Integer; duration: Cardinal): canStatus; stdcall;
  kvSelfTest: function(handle: canHandle; var presults: Int64): canStatus; stdcall;
  kvFlashLeds: function(handle: canHandle; action: Integer; timeout: Integer): canStatus; stdcall;
  canSetBitrate: function(handle: canHandle; bitrate: Integer): canStatus; stdcall;
  canGetHandleData: function(handle: canHandle; item: Integer; var Buffer; bufsize: Int64): canStatus; stdcall;
  kvGetApplicationMapping: function(busType: Integer; appName: PAnsiChar; appChannel: Integer; var resultingChannel: Integer): canStatus; stdcall;
  kvTimeDomainCreate: function(var domain: kvTimeDomain): kvStatus; stdcall;
  kvTimeDomainDelete: function(domain: kvTimeDomain): kvStatus; stdcall;
  kvTimeDomainResetTime: function(domain: kvTimeDomain): kvStatus; stdcall;
  kvTimeDomainGetData: function(domain: kvTimeDomain; var data: kvTimeDomainData; bufsiz: Int64): kvStatus; stdcall;
  kvTimeDomainAddHandle: function(domain: kvTimeDomain; handle: canHandle): kvStatus; stdcall;
  kvTimeDomainRemoveHandle: function(domain: kvTimeDomain; handle: canHandle): kvStatus; stdcall;
  kvReadDeviceCustomerData: function(hnd: canHandle;userNumber, itemNumber: Integer; var data; bufsize: Int64): kvStatus; stdcall;
  kvGetSupportedInterfaceInfo: function(index: Integer; hwName: PAnsiChar; nameLen: Int64; var hwType: Integer; var hwBusType: Integer): kvStatus; stdcall;
  kvScriptStart: function(const hnd: canHandle; slotNo: integer): kvStatus; stdcall;
  kvScriptStatus: function(const hnd: canHandle; slotNo: integer; var status: integer): kvStatus; stdcall;
  kvScriptStop: function(const hnd: canHandle; slotNo: integer; mode: integer): kvStatus; stdcall;
  kvScriptUnload: function(const hnd: canHandle; slotNo: integer): kvStatus; stdcall;
  kvScriptSendEvent: function(const hnd: canHandle;
                              slotNo: integer;
                              eventType: integer;
                              eventNo: integer;
                              data: Cardinal): kvStatus; stdcall;
  kvScriptEnvvarOpen: function(const hnd: canHandle; envvarName: PAnsiChar; var envvarType: Integer; var envvarSize: Integer): kvEnvHandle; stdcall;

  kvScriptEnvvarClose: function(const eHnd: kvEnvHandle): kvStatus; stdcall;
  kvScriptEnvvarSetInt: function(const eHnd: kvEnvHandle; val: Integer): kvStatus; stdcall;
  kvScriptEnvvarGetInt: function(const eHnd: kvEnvHandle; var val: Integer): kvStatus; stdcall;
  kvScriptEnvvarSetFloat: function(const eHnd: kvEnvHandle; val: Single): kvStatus; stdcall;
  kvScriptEnvvarGetFloat: function(const eHnd: kvEnvHandle; var val: Single): kvStatus; stdcall;
  kvScriptEnvvarSetData: function(const eHnd: kvEnvHandle; var buf; start_index: Integer; data_len: Integer): kvStatus; stdcall;
  kvScriptEnvvarGetData: function(const eHnd: kvEnvHandle; var buf; start_index: Integer; data_len: Integer): kvStatus; stdcall;
  kvScriptGetMaxEnvvarSize: function(hnd: canHandle; var envvarSize: Integer): kvStatus; stdcall;
  kvScriptLoadFileOnDevice: function(hnd: canHandle; slotNo: Integer; localFile: PAnsiChar): kvStatus; stdcall;
  kvScriptLoadFile: function(hnd: canHandle; slotNo: Integer; filePathOnPC: PAnsiChar): kvStatus; stdcall;
  kvScriptRequestText: function(hnd: canHandle; slotNo: cardinal; request: cardinal): kvStatus; stdcall;
  kvScriptGetText: function(hnd: canHandle; var slot: integer; var time: Int64; var flags: Cardinal; buf: PAnsiChar; bufsize: Int64): kvStatus; stdcall;
  kvFileCopyToDevice: function(hnd: canHandle; hostFileName: PAnsiChar; deviceFileName: PAnsiChar): kvStatus; stdcall;
  kvFileCopyFromDevice: function(hnd: canHandle; deviceFileName: PAnsiChar; hostFileName: PAnsiChar): kvStatus; stdcall;
  kvFileDelete: function(hnd: canHandle; deviceFileName: PAnsiChar): kvStatus; stdcall;
  kvFileGetName: function(hnd: canHandle; fileNo: Integer; name: PAnsiChar; namelen: Integer): kvStatus; stdcall;
  kvFileGetCount: function(hnd: canHandle; var count: Integer): kvStatus; stdcall;
  kvFileGetSystemData: function(hnd: canHandle; itemCode: Integer; var result: Integer): kvStatus; stdcall;
  kvDeviceSetMode: function(hnd: canHandle; mode: Integer): kvStatus; stdcall;
  kvDeviceGetMode: function(hnd: canHandle; var mode: Integer): kvStatus; stdcall;
  kvPingRequest: function(hnd: canHandle; var requestTime: Cardinal): kvStatus; stdcall;
  kvPingGetLatest: function(hnd: canHandle; var requestTime: Cardinal; var pingTime: Cardinal): kvStatus; stdcall;

const

  kvLED_ACTION_ALL_LEDS_ON    = 0;
  kvLED_ACTION_ALL_LEDS_OFF   = 1;
  kvLED_ACTION_LED_0_ON       = 2;
  kvLED_ACTION_LED_0_OFF      = 3;
  kvLED_ACTION_LED_1_ON       = 4;
  kvLED_ACTION_LED_1_OFF      = 5;
  kvLED_ACTION_LED_2_ON       = 6;
  kvLED_ACTION_LED_2_OFF      = 7;
  kvLED_ACTION_LED_3_ON       = 8;
  kvLED_ACTION_LED_3_OFF      = 9;

  canCHANNELDATA_CHANNEL_CAP = 1;
  canCHANNELDATA_TRANS_CAP = 2;
  canCHANNELDATA_CHANNEL_FLAGS = 3; // available, etc
  canCHANNELDATA_CARD_TYPE = 4; // canHWTYPE_xxx
  canCHANNELDATA_CARD_NUMBER = 5; // Number in machine, 0,1,...
  canCHANNELDATA_CHAN_NO_ON_CARD = 6;
  canCHANNELDATA_CARD_SERIAL_NO = 7;
  canCHANNELDATA_TRANS_SERIAL_NO = 8;
  canCHANNELDATA_CARD_FIRMWARE_REV = 9;
  canCHANNELDATA_CARD_HARDWARE_REV = 10;
  canCHANNELDATA_CARD_UPC_NO = 11;
  canCHANNELDATA_TRANS_UPC_NO = 12;
  canCHANNELDATA_CHANNEL_NAME = 13;
  canCHANNELDATA_DLL_FILE_VERSION = 14;
  canCHANNELDATA_DLL_PRODUCT_VERSION = 15;
  canCHANNELDATA_DLL_FILETYPE = 16;
  canCHANNELDATA_TRANS_TYPE = 17;
  canCHANNELDATA_DEVICE_PHYSICAL_POSITION = 18;
  canCHANNELDATA_UI_NUMBER = 19;
  canCHANNELDATA_TIMESYNC_ENABLED = 20;
  canCHANNELDATA_DRIVER_FILE_VERSION = 21;
  canCHANNELDATA_DRIVER_PRODUCT_VERSION = 22;
  canCHANNELDATA_MFGNAME_UNICODE = 23;
  canCHANNELDATA_MFGNAME_ASCII = 24;
  canCHANNELDATA_DEVDESCR_UNICODE = 25;
  canCHANNELDATA_DEVDESCR_ASCII = 26;
  canCHANNELDATA_DRIVER_NAME = 27;
  canCHANNELDATA_CHANNEL_QUALITY = 28;
  canCHANNELDATA_ROUNDTRIP_TIME = 29;
  canCHANNELDATA_BUS_TYPE = 30;
  canCHANNELDATA_DEVNAME_ASCII = 31;
  canCHANNELDATA_TIME_SINCE_LAST_SEEN = 32;
  canCHANNELDATA_REMOTE_OPERATIONAL_MODE = 33;
  canCHANNELDATA_REMOTE_PROFILE_NAME = 34;
  canCHANNELDATA_REMOTE_HOST_NAME = 35;
  canCHANNELDATA_REMOTE_MAC = 36;
  canCHANNELDATA_MAX_BITRATE = 37;
  canCHANNELDATA_CHANNEL_CAP_MASK = 38;


// channelFlags in canChannelData
  canCHANNEL_IS_EXCLUSIVE = $0001;
  canCHANNEL_IS_OPEN = $0002;

// For canOpen(), canOpenChannel()
  canWANT_EXCLUSIVE = $08; { Don't allow sharing }
  canWANT_EXTENDED = $10; { Extended CAN is required }
  canWANT_VIRTUAL = $0020;
  canOPEN_EXCLUSIVE           = canWANT_EXCLUSIVE;
  canOPEN_REQUIRE_EXTENDED    = canWANT_EXTENDED;
  canOPEN_ACCEPT_VIRTUAL      = canWANT_VIRTUAL;
  canOPEN_OVERRIDE_EXCLUSIVE  = $0040;
  canOPEN_REQUIRE_INIT_ACCESS = $0080;
  canOPEN_NO_INIT_ACCESS      = $0100;
  canOPEN_ACCEPT_LARGE_DLC    = $0200;
  canOPEN_CAN_FD              = $0400;
  canOPEN_CAN_FD_NONISO       = $0800;


// Hardware types.
  canHWTYPE_NONE = 0; // Unknown
  canHWTYPE_VIRTUAL = 1; // Virtual channel.
  canHWTYPE_LAPCAN = 2; // LAPcan family
  canHWTYPE_CANPARI = 3; // CANpari (not supported.)
  canHWTYPE_PCCAN = 8; // PCcan family
  canHWTYPE_PCICAN = 9; // PCIcan family
  canHWTYPE_USBCAN = 11; // USBcan family
  canHWTYPE_PCICAN_II = 40;
  canHWTYPE_USBCAN_II = 42;
  canHWTYPE_SIMULATED = 44;
  canHWTYPE_ACQUISITOR = 46;
  canHWTYPE_LEAF = 48;
  canHWTYPE_PC104_PLUS = 50;      // PC104+
  canHWTYPE_PCICANX_II = 52;      // PCIcanx II
  canHWTYPE_MEMORATOR_II = 54;    // Memorator Professional
  canHWTYPE_MEMORATOR_PRO = 54;   // Memorator Professional
  canHWTYPE_USBCAN_PRO = 56;      // USBcan Professional
  canHWTYPE_IRIS = 58;            // Obsolete name, use canHWTYPE_BLACKBIRD instead
  canHWTYPE_BLACKBIRD = 58;
  canHWTYPE_MEMORATOR_LIGHT = 60; ///< Kvaser Memorator Light
  canHWTYPE_MINIHYDRA = 62;       ///< Obsolete name, use canHWTYPE_EAGLE instead
  canHWTYPE_EAGLE = 62;           ///< Kvaser Eagle family
  canHWTYPE_BAGEL = 64;           ///< Obsolete name, use canHWTYPE_BLACKBIRD_V2 instead
  canHWTYPE_BLACKBIRD_V2 = 64;    ///< Kvaser BlackBird v2
  canHWTYPE_MINIPCIE = 66;        ///< "Mini PCI Express" for now, subject to change.
  canHWTYPE_USBCAN_KLINE = 68;    ///< USBcan Pro HS/K-Line
  canHWTYPE_ETHERCAN = 70;        ///< Kvaser Ethercan
  canHWTYPE_USBCAN_LIGHT = 72;    ///< Kvaser USBcan Light
  canHWTYPE_USBCAN_PRO2 = 74;     ///< Kvaser USBcan Pro 5xHS
  canHWTYPE_PCIE_V2 = 76;         ///< PCIe for now
  canHWTYPE_MEMORATOR_PRO2 = 78;  ///< Kvaser Memorator Pro 5xHS
  canHWTYPE_LEAF2 = 80;           ///< Kvaser Leaf Pro HS v2 and variants
  canHWTYPE_MEMORATOR_V2 = 82;    ///< Kvaser Memorator (2nd generation)


  canTRANSCEIVER_TYPE_UNKNOWN = 0;
  canTRANSCEIVER_TYPE_251 = 1;
  canTRANSCEIVER_TYPE_252 = 2;
  canTRANSCEIVER_TYPE_DNOPTO = 3;
  canTRANSCEIVER_TYPE_W210 = 4;
  canTRANSCEIVER_TYPE_SWC_PROTO = 5;
  canTRANSCEIVER_TYPE_SWC = 6;
  canTRANSCEIVER_TYPE_EVA = 7;
  canTRANSCEIVER_TYPE_FIBER = 8;
  canTRANSCEIVER_TYPE_K251 = 9;
  canTRANSCEIVER_TYPE_K = 10;
  canTRANSCEIVER_TYPE_1054_OPTO = 11;
  canTRANSCEIVER_TYPE_SWC_OPTO = 12;
  canTRANSCEIVER_TYPE_TT = 13;
  canTRANSCEIVER_TYPE_1050 = 14;
  canTRANSCEIVER_TYPE_1050_OPTO = 15;
  canTRANSCEIVER_TYPE_1041 = 16;
  canTRANSCEIVER_TYPE_1041_OPTO = 17;
  canTRANSCEIVER_TYPE_RS485 = 18;
  canTRANSCEIVER_TYPE_LIN = 19;
  canTRANSCEIVER_TYPE_KONE = 20;
  canTRANSCEIVER_TYPE_CANFD = 22;
  canTRANSCEIVER_TYPE_LINX_LIN       = 64;
  canTRANSCEIVER_TYPE_LINX_J1708     = 66;
  canTRANSCEIVER_TYPE_LINX_K         = 68;
  canTRANSCEIVER_TYPE_LINX_SWC       = 70;
  canTRANSCEIVER_TYPE_LINX_LS        = 72;


// Channel capabilities.
  canCHANNEL_CAP_EXTENDED_CAN = $00000001;
  canCHANNEL_CAP_BUS_STATISTICS = $00000002;
  canCHANNEL_CAP_ERROR_COUNTERS = $00000004;
  canCHANNEL_CAP_CAN_DIAGNOSTICS = $00000008;
  canCHANNEL_CAP_GENERATE_ERROR = $00000010;
  canCHANNEL_CAP_GENERATE_OVERLOAD = $00000020;
  canCHANNEL_CAP_TXREQUEST = $00000040;
  canCHANNEL_CAP_TXACKNOWLEDGE = $00000080;
  canCHANNEL_CAP_VIRTUAL = $00010000;
  canCHANNEL_CAP_SIMULATED = $00020000;
  canCHANNEL_CAP_REMOTE = $00040000;
  canCHANNEL_CAP_CAN_FD = $00080000;
  canCHANNEL_CAP_CAN_FD_NONISO = $00100000;

// Driver (transceiver) capabilities
  canDRIVER_CAP_HIGHSPEED = $00000001;


implementation

uses
  SysUtils;

var
  hDLL: THandle;
  //realCanLocateHardware: function: canStatus; stdcall;
  realCanInitializeLibrary: procedure;
  //realCanUnloadLibrary: function: Integer;
  DLLName: array[0..50] of char = 'libcanlib.so';


Constructor TCanInterface.create(canChannel: Integer) overload;
Var
  cname: packed array[0..256] of AnsiChar;
begin
  channel := canChannel;

  FillChar(cname, Sizeof(cname), #0);
  canGetChannelData(channel, canCHANNELDATA_CHANNEL_NAME, cname, 256);
  //OutputDebugString(PCHAR(inttostr(status)));
  name := Format('%s', [cname]);
  //Inherited Create;
end;



procedure LoadDLL; forward;
procedure UnloadDLL; forward;

procedure canInitializeLibrary; stdcall;
begin
  if hDLL <> 0 then Exit;
  LoadDLL;
  if hDLL <> 0 then begin
    realCanInitializeLibrary;
  end;
end;

//function canLocateHardware: canStatus; stdcall;
//begin
//  if hDLL <> 0 then begin
//    Result := canOK;
//    Exit;
//  end;
//  LoadDLL;
//  if hDLL = 0 then begin
//    Result := canERR_DYNALOAD;
//  end else begin
//    Result := realCanLocateHardware();
//  end;
//end;

function canUnloadLibrary: Integer; stdcall;
begin
  if hDLL = 0 then begin
    Result := canOK;
    Exit;
  end;
  //if Assigned(realCanUnloadLibrary) then realCanUnloadLibrary;
  UnloadDLL;
  Result := canOK;
end;


function GPA(const proc: string): Pointer;
var s: array[0..300] of char;
begin
  StrPCopy(s, proc);
  Result := GetProcAddress(hDLL, s);
  if Result = nil then begin
    raise Exception.CreateFmt('CANLIB: function %s not found.', [proc]);
  end;
end;

procedure SetDllName(s: string);
begin
  StrPCopy(DLLName, s);
end;

procedure LoadDLL;
//var
//  err: integer;
begin
  hDLL := LoadLibrary(DLLName);
  //err := GetLastError;
  if hDLL = 0 then begin
    raise Exception.Create('Can not load the CAN driver - is it correctly installed?');
    Exit;
  end;

  //@realCanLocateHardware := GPA('canLocateHardware');
  @realCanInitializeLibrary := GPA('canInitializeLibrary');

  //@canOpen := GPA('canOpen');

  @canClose := GPA('canClose');
  @canBusOn := GPA('canBusOn');
  @canBusOff := GPA('canBusOff');
  @canSetBusParams := GPA('canSetBusParams');
  @canGetBusParams := GPA('canGetBusParams');
  @canSetBusParamsFd := GPA('canSetBusParamsFd');
  @canGetBusParamsFd := GPA('canGetBusParamsFd');
  @canSetBusOutputControl := GPA('canSetBusOutputControl');
  @canGetBusOutputControl := GPA('canGetBusOutputControl');
  @canAccept := GPA('canAccept');
  @canReadStatus := GPA('canReadStatus');
  @canReadErrorCounters := GPA('canReadErrorCounters');
  @canWrite := GPA('canWrite');
  @canWriteSync := GPA('canWriteSync');
  @canRead := GPA('canRead');
  @canReadWait := GPA('canReadWait');
  //@canReadSpecific := GPA('canReadSpecific');
  @canReadSync := GPA('canReadSync');
  //@canReadSyncSpecific := GPA('canReadSyncSpecific');
  //@canReadSpecificSkip := GPA('canReadSpecificSkip');
  //@canInstallAction := nil;
  //@canUninstallAction := nil;
  //@canInstallOwnBuffer := nil;
  //@canUninstallOwnBuffer := nil;
//  @canSetNotify := GPA('canSetNotify');
  @canTranslateBaud := GPA('canTranslateBaud');
  @canGetErrorText := GPA('canGetErrorText');
  @canGetVersion := GPA('canGetVersion');
  @canIoCtl := GPA('canIoCtl');
  @canReadTimer := GPA('canReadTimer');
  @canGetNumberOfChannels := GPA('canGetNumberOfChannels');
  @canGetChannelData := GPA('canGetChannelData');
  @canOpenChannel := GPA('canOpenChannel');
  //@canWaitForEvent := GPA('canWaitForEvent');
  @canSetBusParamsC200 := GPA('canSetBusParamsC200');
  //@canGetVersionEx := GPA('canGetVersionEx');
  //@canSetDriverMode := GPA('canSetDriverMode');
  //@canGetDriverMode := GPA('canGetDriverMode');
  //@canParamGetCount := GPA('canParamGetCount');
  //@canParamCommitChanges := GPA('canParamCommitChanges');
  //@canParamDeleteEntry := GPA('canParamDeleteEntry');
  //@canParamCreateNewEntry := GPA('canParamCreateNewEntry');
  //@canParamSwapEntries := GPA('canParamSwapEntries');
  //@canParamGetName := GPA('canParamGetName');
  //@canParamGetChannelNumber := GPA('canParamGetChannelNumber');
  //@canParamGetBusParams := GPA('canGetBusParams');
  //@canParamSetName := GPA('canParamSetName');
  //@canParamSetChannelNumber := GPA('canParamSetChannelNumber');
  //@canParamSetBusParams := GPA('canParamSetBusParams');
  //@canParamFindByName := GPA('canParamFindByName');
  @canObjBufFreeAll := GPA('canObjBufFreeAll');
  @canObjBufAllocate := GPA('canObjBufAllocate');
  @canObjBufFree := GPA('canObjBufFree');
  @canObjBufWrite := GPA('canObjBufWrite');
  @canObjBufSetFilter := GPA('canObjBufSetFilter');
  @canObjBufSetFlags := GPA('canObjBufSetFilter');
  @canObjBufEnable := GPA('canObjBufEnable');
  @canObjBufDisable := GPA('canObjBufDisable');
  //@canProbeVersion := GPA('canProbeVersion');
  @canResetBus := GPA('canResetBus');
  @canWriteWait := GPA('canWriteWait');
  //@canSetAcceptanceFilter :=GPA('canSetAcceptanceFilter');
  //@canRequestChipStatus := GPA('canRequestChipStatus');
  //@canRequestBusStatistics := GPA('canRequestBusStatistics');
  //@canGetBusStatistics := GPA('canGetBusStatistics');
  //@kvAnnounceIdentity := GPA('kvAnnounceIdentity');
  //@kvSetNotifyCallback := GPA('kvSetNotifyCallback');
  //@kvBeep := GPA('kvBeep');
  //@kvSelfTest := GPA('kvSelfTest');
  //@kvFlashLeds := GPA('kvFlashLeds');
  //@canSetBitrate := GPA('canSetBitrate');
  @canGetHandleData := GPA('canGetHandleData');
  //@kvTimeDomainCreate := GPA('kvTimeDomainCreate');
  //@kvTimeDomainDelete := GPA('kvTimeDomainDelete');
  //@kvTimeDomainResetTime := GPA('kvTimeDomainResetTime');
  //@kvTimeDomainGetData := GPA('kvTimeDomainGetData');
  //@kvTimeDomainAddHandle := GPA('kvTimeDomainAddHandle');
  //@kvTimeDomainRemoveHandle := GPA('kvTimeDomainRemoveHandle');
  //@kvReadDeviceCustomerData := GPA('kvReadDeviceCustomerData');
  //@kvReadTimer := GPA('kvReadTimer');
  //@kvReadTimer64 := GPA('kvReadTimer64');
  @canObjBufSetPeriod := GPA('canObjBufSetPeriod');
  @canObjBufSetMsgCount := GPA('canObjBufSetMsgCount');
  @canObjBufSendBurst := GPA('canObjBufSendBurst');
  @canFlushReceiveQueue := GPA('canFlushReceiveQueue');
  @canFlushTransmitQueue := GPA('canFlushTransmitQueue');
  //@kvAnnounceIdentityEx := GPA('kvAnnounceIdentityEx');
  //@kvGetApplicationMapping := GPA('kvGetApplicationMapping');
  //@kvGetSupportedInterfaceInfo := GPA('kvGetSupportedInterfaceInfo');
  //@kvScriptStart := GPA('kvScriptStart');
  //@kvScriptStatus := GPA('kvScriptStatus');
  //@kvScriptStop := GPA('kvScriptStop');
  //@kvScriptUnload := GPA('kvScriptUnload');
  //@kvScriptSendEvent := GPA('kvScriptSendEvent');
  //@kvScriptEnvvarOpen := GPA('kvScriptEnvvarOpen');
  //@kvScriptEnvvarClose := GPA('kvScriptEnvvarClose');
  //@kvScriptEnvvarSetInt := GPA('kvScriptEnvvarSetInt');
  //@kvScriptEnvvarGetInt := GPA('kvScriptEnvvarGetInt');
  //@kvScriptEnvvarSetFloat := GPA('kvScriptEnvvarSetFloat');
  //@kvScriptEnvvarGetFloat := GPA('kvScriptEnvvarGetFloat');
  //@kvScriptEnvvarSetData := GPA('kvScriptEnvvarSetData');
  //@kvScriptEnvvarGetData := GPA('kvScriptEnvvarGetData');
  //@kvScriptGetMaxEnvvarSize := GPA('kvScriptGetMaxEnvvarSize');
  //@kvScriptLoadFileOnDevice := GPA('kvScriptLoadFileOnDevice');
  //@kvScriptLoadFile := GPA('kvScriptLoadFile');
  //@kvScriptRequestText := GPA('kvScriptRequestText');
  //@kvScriptGetText := GPA('kvScriptGetText');
  //@kvFileCopyToDevice := GPA('kvFileCopyToDevice');
  //@kvFileCopyFromDevice := GPA('kvFileCopyFromDevice');
  //@kvFileDelete := GPA('kvFileDelete');
  //@kvFileGetName := GPA('kvFileGetName');
  //@kvFileGetCount := GPA('kvFileGetCount');
  //@kvFileGetSystemData := GPA('kvFileGetSystemData');
  //@kvDeviceSetMode := GPA('kvDeviceSetMode');
  //@kvDeviceGetMode := GPA('kvDeviceGetMode');
  //@kvPingRequest := GPA('kvPingRequest');
  //@kvPingGetLatest := GPA('kvPingGetLatest');
  {--}
  //@realCanUnloadLibrary := GPA('canUnloadLibrary');

end;

procedure UnloadDLL;
begin
  //if not Assigned(realCanUnloadLibrary) then Exit;
  //realCanUnloadLibrary;
  FreeLibrary(hDLL);
  hDLL := 0;
end;


end.
