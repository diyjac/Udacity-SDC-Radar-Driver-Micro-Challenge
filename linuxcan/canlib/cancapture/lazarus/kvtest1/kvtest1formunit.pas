unit kvtest1formunit;

{$mode objfpc}{$H+}

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

interface

uses
  Classes, SysUtils, FileUtil, Forms, Controls, Graphics, Dialogs, StdCtrls,
  Buttons, ActnList, ExtCtrls, Canlib;

type

  { TKvTest1Form }

  TKvTest1Form = class(TForm)
    BChannelSelectComboBox: TComboBox;
    ConfigSilentModeCheckBox: TCheckBox;
    ConfigBusOnOffAction: TAction;
    ConfigActionList: TActionList;
    ConfigBusOnBitBtn: TBitBtn;
    AChannelSelectComboBox: TComboBox;
    ConfigBusOffBitBtn: TBitBtn;
    CanFdCheckBox: TCheckBox;
    GroupBox1: TGroupBox;
    Label1: TLabel;
    Label2: TLabel;
    RxDataMemo: TMemo;
    RxDataTimer: TTimer;
    TxDataTimer: TTimer;
    procedure ConfigBusOnOffActionExecute(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure RxDataTimerTimer(Sender: TObject);
    procedure TxDataTimerTimer(Sender: TObject);
  private
    { private declarations }
    isCanBusOn: Boolean;
    canAChannel: Integer;
    canBChannel: Integer;
    canHandleRead : Integer;
    canHandleWrite : Integer;
    canOpenFlags: Cardinal;
    RxData: Array[0..127] of Byte;
    TxData: Array[0..127] of Byte;
    procedure BuildChannelList;
    procedure GoBusOn;
    procedure GoBusOff;
    procedure InitConfig;
    procedure HandleRxData;
  public
    { public declarations }
  end;

  const
    MAX_DATA_LENGTH = 64;
    canINVALID_CHANNEL : Integer = -1;

var
  KvTest1Form: TKvTest1Form;

implementation

{$R *.lfm}

procedure TKvTest1Form.FormShow(Sender: TObject);
begin
  BuildChannelList;
end;

procedure TKvTest1Form.RxDataTimerTimer(Sender: TObject);
begin
  HandleRxData;
end;

procedure TKvTest1Form.TxDataTimerTimer(Sender: TObject);
var
  status: canStatus;
  id: Int64;
  dlc: Cardinal;
  flags: Cardinal;
  i: Cardinal;
begin
  id := 42;
  for i := Low(TxData) to MAX_DATA_LENGTH - 1 do begin
    TxData[i] := i;
  end;
  dlc := 8;
  flags := canMSG_EXT;
  if CanFdCheckBox.Checked then begin
    flags := flags or canFDMSG_FDF or canFDMSG_BRS;
  end;
  try
    status := canWrite(canHandleWrite, id, @TxData, dlc, flags);
    if Status <> canOk then begin
      TxDataTimer.Enabled := False;
      MessageDlg('Failed to send frame. Shutting off Tx. Status: ' + IntToStr(status), mtError, [mbOk], 0);
    end;
  except
    TxDataTimer.Enabled := False;
  end;
end;

procedure TKvTest1Form.ConfigBusOnOffActionExecute(Sender: TObject);
begin
  if not isCanBusOn then begin
    GoBusOn;
  end
  else begin
    GoBusOff;
  end;
  ConfigBusOffBitBtn.Visible := isCanBusOn;
  ConfigBusOnBitBtn.Visible := not isCanBusOn;
  RxDataTimer.Enabled:= isCanBusOn;
  TxDataTimer.Enabled:= isCanBusOn;
  if isCanBusOn then
    RxDataMemo.Lines.Add('Bus on')
  else
    RxDataMemo.Lines.Add('Bus off');
end;

procedure TKvTest1Form.FormCreate(Sender: TObject);
begin
  ConfigBusOffBitBtn.Top := ConfigBusOnBitBtn.Top;
  ConfigBusOffBitBtn.Left := ConfigBusOnBitBtn.Left;
  InitConfig;
end;

procedure TKvTest1Form.BuildChannelList;
{ Enumerate all CAN channels and populate the drop-down boxes }
var
  i: Integer;
  numChannels: Integer = 0;
  s: string;
  p: packed Array[0..64] of char;
  n: Integer = 0;
begin
  FillChar(p, Sizeof(p), #0);
  canUnloadLibrary;
  canInitializeLibrary;

  canGetNumberOfChannels(numChannels);
  AChannelSelectComboBox.Items.Clear;
  BChannelSelectComboBox.Items.Clear;
  try
    Screen.Cursor := crHourGlass;
    AChannelSelectComboBox.Items.BeginUpdate;
    BChannelSelectComboBox.Items.BeginUpdate;
    for i := 0 to numChannels - 1 do begin
      canGetChannelData(i, canCHANNELDATA_DEVDESCR_ASCII, p, sizeof(p));
      canGetChannelData(i, canCHANNELDATA_CHAN_NO_ON_CARD, n, sizeof(n));
      s := Format('[%d] %s channel %d', [i, string(p), n + 1]);
      AChannelSelectComboBox.Items.Add(s);
      BChannelSelectComboBox.Items.Add(s);
    end;
  finally
    Screen.Cursor := crDefault;
  end;
  AChannelSelectComboBox.Items.EndUpdate;
  BChannelSelectComboBox.Items.EndUpdate;
  if AChannelSelectComboBox.Items.Count = 0 then begin
    ConfigBusOnBitBtn.Enabled := False;
  end;
  if numChannels > 0 then begin
    AChannelSelectComboBox.ItemIndex := 0;
    if numChannels > 1 then
      BChannelSelectComboBox.ItemIndex := 1
    else
      BChannelSelectComboBox.ItemIndex := 0;
  end;
end;

procedure TKvTest1Form.GoBusOn;
var
  status: canStatus;
  flags: Integer;
  driverType: Cardinal;
const
  txechoControl : Cardinal = 1;
label
  ErrorExit;
begin
  status := canOk;
  if not isCanBusOn then begin
    canAChannel := AChannelSelectComboBox.ItemIndex;
    canBChannel := BChannelSelectComboBox.ItemIndex;
    flags := canOPEN_ACCEPT_VIRTUAL or canOPEN_EXCLUSIVE or canOPEN_REQUIRE_EXTENDED;
    if CanFdCheckBox.Checked then begin
      flags := flags or canOPEN_CAN_FD;
    end;
    canHandleRead := canOpenChannel(canAChannel, flags);
    if canHandleRead < 0 then begin
      status := canHandleRead;
      Dialogs.MessageDlg('Failed to open channel #' + IntToStr(canAChannel), mtError, [mbOk], 0);
      goto ErrorExit;
    end;
    canHandleWrite := canOpenChannel(canBChannel, flags);
    if canHandleWrite < 0 then begin
      status := canHandleWrite;
      Dialogs.MessageDlg('Failed to open channel #' + IntToStr(canBChannel), mtError, [mbOk], 0);
      goto ErrorExit;
    end;
    if (flags and (canOPEN_CAN_FD or canOPEN_CAN_FD_NONISO)) <> 0 then begin
      status := canSetBusParams(canHandleRead, canFD_BITRATE_500K_80P, 0, 0, 0, 0, 0);
      if Status <> canOk then begin
        MessageDlg('Failed to set bus parameters', mtError, [mbOk], 0);
        goto ErrorExit;
      end;
      status := canSetBusParams(canHandleWrite, canFD_BITRATE_500K_80P, 0, 0, 0, 0, 0);
      if Status <> canOk then begin
        MessageDlg('Failed to set bus parameters', mtError, [mbOk], 0);
        goto ErrorExit;
      end;

      status := canSetBusParamsFd(canHandleRead, canFD_BITRATE_2M_80P, 0, 0, 0);
      if Status <> canOk then begin
        MessageDlg('Failed to set bus FD parameters', mtError, [mbOk], 0);
        goto ErrorExit;
      end;
      status := canSetBusParamsFd(canHandleWrite, canFD_BITRATE_2M_80P, 0, 0, 0);
      if Status <> canOk then begin
        MessageDlg('Failed to set bus FD parameters', mtError, [mbOk], 0);
        goto ErrorExit;
      end;
    end
    else begin
      status := canSetBusParams(canHandleRead, canBITRATE_500K, 0, 0, 0, 0, 0);
      if Status <> canOk then begin
        MessageDlg('Failed to set bus parameters', mtError, [mbOk], 0);
        goto ErrorExit;
      end;
      status := canSetBusParams(canHandleWrite, canBITRATE_500K, 0, 0, 0, 0, 0);
      if Status <> canOk then begin
        MessageDlg('Failed to set bus parameters', mtError, [mbOk], 0);
        goto ErrorExit;
      end;
    end;

    status := canIoCtl(canHandleRead, canIOCTL_SET_LOCAL_TXECHO, @txechoControl, Sizeof(txechoControl));
    if status <> canOK then begin
      MessageDlg('Failed to set canIOCTL_SET_LOCAL_TXECHO', mtError, [mbOk], 0);
      goto ErrorExit;
    end;
    status := canIoCtl(canHandleWrite, canIOCTL_SET_LOCAL_TXECHO, @txechoControl, Sizeof(txechoControl));
    if status <> canOK then begin
      MessageDlg('Failed to set canIOCTL_SET_LOCAL_TXECHO', mtError, [mbOk], 0);
      goto ErrorExit;
    end;

    if ConfigSilentModeCheckBox.Checked then begin
      driverType := canDRIVER_SILENT;
    end
    else begin
      driverType := canDRIVER_NORMAL
    end;
    status := canSetBusOutputControl(canHandleRead, driverType);
    if Status <> canOk then begin
      MessageDlg('Failed to set bus output control', mtError, [mbOk], 0);
      goto ErrorExit;
    end;
    status := canSetBusOutputControl(canHandleWrite, driverType);
    if Status <> canOk then begin
      MessageDlg('Failed to set bus output control', mtError, [mbOk], 0);
      goto ErrorExit;
    end;

    status := canBusOn(canHandleRead);
    if Status <> canOk then begin
      MessageDlg('Failed to go bus on', mtError, [mbOk], 0);
      goto ErrorExit;
    end;
    status := canBusOn(canHandleWrite);
    if Status <> canOk then begin
      MessageDlg('Failed to go bus on', mtError, [mbOk], 0);
      goto ErrorExit;
    end;
    isCanBusOn := (status = canOk);
    if isCanBusOn then begin
      canOpenFlags := flags;
    end
    else begin
      canOpenFlags := 0;
    end;
  end;

ErrorExit:
  if Status <> canOk then begin
    if canHandleRead >= 0 then
      canClose(canHandleRead);
    if canHandleWrite >= 0 then
      canClose(canHandleWrite);
    canHandleRead := canINVALID_HANDLE;
    canHandleWrite := canINVALID_HANDLE;
    canAChannel := canINVALID_CHANNEL;
    canBChannel := canINVALID_CHANNEL;
  end;
end;

procedure TKvTest1Form.GoBusOff;
begin
  if isCanBusOn then begin
    isCanBusOn := false;
    canBusOff(canHandleRead);
    canBusOff(canHandleWrite);
    canClose(canHandleRead);
    canClose(canHandleWrite);
    canHandleRead := canINVALID_HANDLE;
    canHandleWrite := canINVALID_HANDLE;
    canAChannel := canINVALID_CHANNEL;
    canBChannel := canINVALID_CHANNEL;
  end;
end;

procedure TKvTest1Form.InitConfig;
begin
  canAChannel := canINVALID_CHANNEL;
  canBChannel := canINVALID_CHANNEL;
end;

procedure TKvTest1Form.HandleRxData;
var
  id: Int64 = 0;
  dlc: Cardinal = 0;
  flags: Cardinal = 0;
  time: Int64 = 0;
  str : String;
  i : Integer;
begin
  SetLength(str, 255);
  while canRead(canHandleRead, id, @RxData, dlc, flags, time) = canOK do begin

    case dlc of
      9..12:    dlc := 12;
      13..16:   dlc := 16;
      17..20:   dlc := 20;
      21..24:   dlc := 24;
      25..32:   dlc := 32;
      33..48:   dlc := 48;
      49..64:   dlc := 64;
    end;

    str := Format('%8.8X %2.2d %8.8X', [id, dlc, time]) + ' ';
    for i := Low(RxData) to dlc - 1 do begin
      str := str + IntToHex(RxData[i], 2) + ' ';
    end;
    if not ((flags and canMSG_ERROR_FRAME) = 0) then
      str := str + ' ERROR FRAME';
    if not ((flags and canMSGERR_HW_OVERRUN) = 0) then
      str := str + ' HWOVR';
    if not ((flags and canMSGERR_SW_OVERRUN) = 0) then
      str := str + ' SWOVR';
    if not ((flags and canMSGERR_STUFF) = 0) then
      str := str + ' STUFF';
    if not ((flags and canMSGERR_FORM) = 0) then
      str := str + ' FORM';
    if not ((flags and canMSGERR_CRC) = 0) then
      str := str + ' CRC';
    if not ((flags and canMSGERR_BIT0) = 0) then
      str := str + ' BIT0';
    if not ((flags and canMSGERR_BIT1) = 0) then
      str := str + ' BIT1';
    RxDataMemo.Lines.Add(str);
  end;

end;

end.

