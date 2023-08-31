unit mainfrm;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, XPMan, StdCtrls, CPDrv, ExtCtrls, ComCtrls;

type
  Tmainform = class(TForm)
    XPManifest1: TXPManifest;
    comport: TCommPortDriver;
    Button1: TButton;
    Memo1: TMemo;
    Button3: TButton;
    Timer1: TTimer;
    Button4: TButton;
    GroupBox1: TGroupBox;
    Label1: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    GroupBox2: TGroupBox;
    Label24: TLabel;
    Label25: TLabel;
    Label26: TLabel;
    Label27: TLabel;
    Label28: TLabel;
    Label29: TLabel;
    Button8: TButton;
    GroupBox3: TGroupBox;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label17: TLabel;
    Label18: TLabel;
    Label19: TLabel;
    Label20: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label23: TLabel;
    speedlbl: TLabel;
    acclbl: TLabel;
    speedbar: TTrackBar;
    accbar: TTrackBar;
    Button2: TButton;
    Button5: TButton;
    GroupBox4: TGroupBox;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label30: TLabel;
    Label31: TLabel;
    Label32: TLabel;
    Label33: TLabel;
    Label34: TLabel;
    Label35: TLabel;
    Label36: TLabel;
    Label37: TLabel;
    Label38: TLabel;
    TrackBar1: TTrackBar;
    TrackBar2: TTrackBar;
    Button6: TButton;
    Button7: TButton;
    GroupBox5: TGroupBox;
    GroupBox6: TGroupBox;
    Button9: TButton;
    Button10: TButton;
    Button11: TButton;
    Button12: TButton;
    Label39: TLabel;
    Label40: TLabel;
    Label41: TLabel;
    Label42: TLabel;
    Label43: TLabel;
    TrackBar3: TTrackBar;
    Label44: TLabel;
    Button13: TButton;
    Label45: TLabel;
    Label46: TLabel;
    Label47: TLabel;
    Label48: TLabel;
    Label49: TLabel;
    TrackBar4: TTrackBar;
    Label50: TLabel;
    Button14: TButton;
    Label51: TLabel;
    Label52: TLabel;
    Label53: TLabel;
    ComboBox1: TComboBox;
    ComboBox2: TComboBox;
    procedure Button1Click(Sender: TObject);
    procedure Butto3Click(Sender: TObject);
    procedure Timer1Timer(Sender: TObject);
    procedure Button4Click(Sender: TObject);
    procedure speedbarChange(Sender: TObject);
    procedure accbarChange(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Button5Click(Sender: TObject);
    procedure TrackBar1Change(Sender: TObject);
    procedure TrackBar2Change(Sender: TObject);
    procedure Button6Click(Sender: TObject);
    procedure Button7Click(Sender: TObject);
    procedure Button8Click(Sender: TObject);
    procedure Button9Click(Sender: TObject);
    procedure Button10Click(Sender: TObject);
    procedure Button12Click(Sender: TObject);
    procedure Button11Click(Sender: TObject);
    procedure TrackBar3Change(Sender: TObject);
    procedure Button13Click(Sender: TObject);
    procedure Button14Click(Sender: TObject);
    procedure TrackBar4Change(Sender: TObject);
  private
    { Private declarations }
    RxData: array[0..41] of byte;
    m1_power, m1_torque, m1_speed, m1_voltage: single;
    m2_power, m2_torque, m2_speed, m2_voltage: single;
    function ReceiveAnswer:string;
    function ReceiveData:boolean;
    function SendCommand(command: string):boolean;
    procedure UpdateGUI;
  public
    { Public declarations }
  end;

var
  mainform: Tmainform;

implementation

{$R *.dfm}

function Tmainform.ReceiveAnswer:string;
var
  rx_string:string;
  rx_char:char;
begin
    rx_string:='';

    // read incoming chars until
    repeat
      // try to read a byte -> timeout is set to 1 second
      if not comport.ReadChar(rx_char) then
      begin
        // we did not received anything -> abort
        break;
      end;

      // only attach regular characters of ASCII-table to the string
      if (byte(rx_char)>=32) then
      begin
        // add the received char to the string
        rx_string:=rx_string + rx_char;
      end;
    until (rx_char=#10) or (rx_char=char(0)); // abort if we receive LineFeed (\n) or a NULL

    // return the received string
    result:=rx_string;
end;

function Tmainform.ReceiveData:boolean;
var
  i:integer;
  rx_char:char;
begin
  for i:=0 to length(RxData)-1 do
  begin
    RxData[0]:=0;
  end;

  for i:=0 to (1 + 2*2 + 3*4 + 3)-1 do
  begin
    if comport.ReadChar(rx_char) then
    begin
      RxData[i]:=byte(rx_char);
    end else
    begin
      break;
      result:=false;
    end;
  end;
  result:=true;
end;

function Tmainform.SendCommand(command: string):boolean;
var
  i, command_sum:integer;
  tx_data:string;
  ErrorCheckByte:byte;
begin
  if not comport.Connected then
    Button1.Click;

  comport.FlushBuffers(true, true);

  command_sum:=0;
  for i := 0 to length(command)-1 do
  begin
    command_sum:=command_sum+byte(command[i+1]); // strings in Delphi begin at 1(!!!)
  end;
  ErrorCheckByte:=trunc(command_sum/length(command));

  tx_data:='A' + command + char(ErrorCheckByte) + 'E' + char(13) + char(10);

  comport.SendDataEx(PChar(tx_data), length(tx_data), 1000);
end;

procedure Tmainform.Button1Click(Sender: TObject);
begin
  comport.PortName:='\\.\COM'+inttostr(combobox1.ItemIndex+1);
  comport.BaudRateValue:=strtoint(combobox2.Items[combobox2.ItemIndex]);
  comport.Connect;
  timer1.Enabled:=true;
end;

procedure ReverseBytes(Source, Dest: Pointer; Size: Integer);
begin
  Dest := PByte(NativeUInt(Dest) + Size - 1);
  while (Size > 0) do
  begin
    PByte(Dest)^ := PByte(Source)^;
    Inc(PByte(Source));
    Dec(PByte(Dest));
    Dec(Size);
  end;
end;

procedure Tmainform.Butto3Click(Sender: TObject);
begin
  SendCommand('F0+0001'); // 1 2
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.UpdateGUI;
var
  value:single;
  i:integer;
  w:word;
begin
  SendCommand('V0+0000');
  if ReceiveData then
  begin
    // expecting (1 + 2*2 + 3*4 + 3)=20 bytes
    if ((RxData[0]=65) and (RxData[20-3]=69)) then
    begin
      w:=(RxData[1] shl 8) + RxData[2];
      case w of
        0: label1.Caption:='CTRL_State_Error';
        1: label1.Caption:='CTRL_State_Idle';
        2: label1.Caption:='CTRL_State_OffLine';
        3: label1.Caption:='CTRL_State_OnLine';
        else
          label1.Caption:=inttostr(w);
      end;

      w:=(RxData[3] shl 8) + RxData[4];
      case w of
        0: label2.Caption:='EST_State_Error';
        1: label2.Caption:='EST_State_Idle';
        2: label2.Caption:='EST_State_RoverL';
        3: label2.Caption:='EST_State_Rs';
        4: label2.Caption:='EST_State_RampUp';
        5: label2.Caption:='EST_State_IdRated';
        6: label2.Caption:='EST_State_RatedFlux_OL';
        7: label2.Caption:='EST_State_RatedFlux';
        8: label2.Caption:='EST_State_RampDown';
        9: label2.Caption:='EST_State_LockRotor';
        10: label2.Caption:='EST_State_Ls';
        11: label2.Caption:='EST_State_Rr';
        12: label2.Caption:='EST_State_MotorIdentified';
        13: label2.Caption:='EST_State_OnLine';
        else
          label2.Caption:=inttostr(w);
      end;

      move(RxData[5], value, 4);
      ReverseBytes(@value, @value, 4);
      m1_voltage:=value;
      move(RxData[9], value, 4);
      ReverseBytes(@value, @value, 4);
      m1_speed:=value*1000;
      move(RxData[13], value, 4);
      ReverseBytes(@value, @value, 4);
      m1_torque:=value;
      m1_power:=m1_torque*2*pi*m1_speed/60; // P=M * 2*pi*n/60  M=Nm, n=rpm

      label3.Caption:='Udc = ' + floattostrf(m1_voltage, ffFixed, 15, 3) + ' V';
      label4.Caption:='Speed = ' + floattostrf(m1_speed, ffFixed, 15, 3) + ' rpm';
      label5.Caption:='Torque = ' + floattostrf(m1_torque, ffFixed, 15, 3) + ' Nm';
      label6.Caption:='Power = ' + floattostrf(m1_power, ffFixed, 15, 3) + ' W';
    end else
    begin
      memo1.lines.add('Error in Rx-frame');
      Timer1.Enabled:=false;
    end;
  end else
  begin
    memo1.lines.add('Error on reading data');
    Timer1.Enabled:=false;
  end;

  sleep(100);

  SendCommand('V1+0000');
  if ReceiveData then
  begin
    // expecting (1 + 2*2 + 3*4 + 3)=20 bytes
    if ((RxData[0]=65) and (RxData[20-3]=69)) then
    begin
      w:=(RxData[1] shl 8) + RxData[2];
      case w of
        0: label24.Caption:='CTRL_State_Error';
        1: label24.Caption:='CTRL_State_Idle';
        2: label24.Caption:='CTRL_State_OffLine';
        3: label24.Caption:='CTRL_State_OnLine';
        else
          label24.Caption:=inttostr(w);
      end;

      w:=(RxData[3] shl 8) + RxData[4];
      case w of
        0: label25.Caption:='EST_State_Error';
        1: label25.Caption:='EST_State_Idle';
        2: label25.Caption:='EST_State_RoverL';
        3: label25.Caption:='EST_State_Rs';
        4: label25.Caption:='EST_State_RampUp';
        5: label25.Caption:='EST_State_IdRated';
        6: label25.Caption:='EST_State_RatedFlux_OL';
        7: label25.Caption:='EST_State_RatedFlux';
        8: label25.Caption:='EST_State_RampDown';
        9: label25.Caption:='EST_State_LockRotor';
        10: label25.Caption:='EST_State_Ls';
        11: label25.Caption:='EST_State_Rr';
        12: label25.Caption:='EST_State_MotorIdentified';
        13: label25.Caption:='EST_State_OnLine';
        else
          label25.Caption:=inttostr(w);
      end;

      move(RxData[5], value, 4);
      ReverseBytes(@value, @value, 4);
      m2_voltage:=value;
      move(RxData[9], value, 4);
      ReverseBytes(@value, @value, 4);
      m2_speed:=value*1000;
      move(RxData[13], value, 4);
      ReverseBytes(@value, @value, 4);
      m2_torque:=value;
      m2_power:=m2_torque*2*pi*m2_speed/60;

      label26.Caption:='Udc = ' + floattostrf(m2_voltage, ffFixed, 15, 3) + ' V';
      label27.Caption:='Speed = ' + floattostrf(m2_speed, ffFixed, 15, 3) + ' rpm';
      label28.Caption:='Torque = ' + floattostrf(m2_torque, ffFixed, 15, 3) + ' Nm';
      label29.Caption:='Power = ' + floattostrf(m2_power, ffFixed, 15, 3) + ' W';
    end else
    begin
      memo1.lines.add('Error in Rx-frame');
      Timer1.Enabled:=false;
    end;
  end else
  begin
    memo1.lines.add('Error on reading data');
    Timer1.Enabled:=false;
  end;
end;


procedure Tmainform.Timer1Timer(Sender: TObject);
begin
  UpdateGUI;
end;

procedure Tmainform.Button4Click(Sender: TObject);
begin
  SendCommand('F0+0000');
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.speedbarChange(Sender: TObject);
var
  valuestr:string;
  value_array:array[0..3] of byte;
  value:single;
  reverse:bool;
begin
{
  // sending ASCII-command
  reverse:=speedbar.position<0;
  valuestr:=inttostr(abs(speedbar.Position));
  if length(valuestr)=1 then
    valuestr:='000'+valuestr
  else if length(valuestr)=2 then
    valuestr:='00'+valuestr
  else if length(valuestr)=3 then
    valuestr:='0'+valuestr;

  if reverse then
    SendCommand('S0-' + valuestr)
  else
    SendCommand('S0+' + valuestr);
}

  // sending float-command
  value:=speedbar.position; // copy speed-value to single-variable
  move(value, value_array[0], 4); // copy bytes into byte-array
  SendCommand('s0+' + char(value_array[3]) + char(value_array[2]) + char(value_array[1]) + char(value_array[0]));

  speedlbl.Caption:= inttostr(speedbar.Position) + ' rpm';

  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.accbarChange(Sender: TObject);
var
  valuestr:string;
begin
  valuestr:=inttostr(accbar.Position);
  if length(valuestr)=1 then
    valuestr:='000'+valuestr
  else if length(valuestr)=2 then
    valuestr:='00'+valuestr
  else if length(valuestr)=3 then
    valuestr:='0'+valuestr;

  acclbl.Caption:= inttostr(accbar.Position) + ' rpm/s';

  SendCommand('A0+' + valuestr);
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.Button2Click(Sender: TObject);
begin
  speedbar.Position:=0;
end;

procedure Tmainform.Button5Click(Sender: TObject);
begin
  accbar.Position:=500;
end;

procedure Tmainform.TrackBar1Change(Sender: TObject);
var
  valuestr:string;
  value:single;
  reverse:bool;
begin
  // sending ASCII-command
  reverse:=trackbar1.position<0;
  valuestr:=inttostr(abs(trackbar1.Position));
  if length(valuestr)=1 then
    valuestr:='000'+valuestr
  else if length(valuestr)=2 then
    valuestr:='00'+valuestr
  else if length(valuestr)=3 then
    valuestr:='0'+valuestr;

  if reverse then
    SendCommand('S1-' + valuestr)
  else
    SendCommand('S1+' + valuestr);

  label37.Caption:= inttostr(trackbar1.Position) + ' rpm';

  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.TrackBar2Change(Sender: TObject);
var
  valuestr:string;
begin
  valuestr:=inttostr(trackbar2.Position);
  if length(valuestr)=1 then
    valuestr:='000'+valuestr
  else if length(valuestr)=2 then
    valuestr:='00'+valuestr
  else if length(valuestr)=3 then
    valuestr:='0'+valuestr;

  label38.Caption:= inttostr(trackbar2.Position) + ' rpm/s';

  SendCommand('A1+' + valuestr);
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.Button6Click(Sender: TObject);
begin
  trackbar1.Position:=0;
end;

procedure Tmainform.Button7Click(Sender: TObject);
begin
  trackbar2.Position:=500;
end;

procedure Tmainform.Button8Click(Sender: TObject);
begin
  SendCommand('F0+0003'); // 1 2
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.Button9Click(Sender: TObject);
begin
  SendCommand('T0+0000');
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.Button10Click(Sender: TObject);
begin
  SendCommand('T1+0000');
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.Button12Click(Sender: TObject);
begin
  SendCommand('T1+0001');
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.Button11Click(Sender: TObject);
begin
  SendCommand('T0+0001');
  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.TrackBar3Change(Sender: TObject);
var
  valuestr:string;
  value:single;
  reverse:bool;
begin
  // sending ASCII-command
  reverse:=trackbar3.position<0;
  valuestr:=inttostr(abs(trackbar3.Position));
  if length(valuestr)=1 then
    valuestr:='000'+valuestr
  else if length(valuestr)=2 then
    valuestr:='00'+valuestr
  else if length(valuestr)=3 then
    valuestr:='0'+valuestr;

  if reverse then
    SendCommand('Q0-' + valuestr)
  else
    SendCommand('Q0+' + valuestr);

  memo1.Lines.Add(ReceiveAnswer);
end;

procedure Tmainform.Button13Click(Sender: TObject);
begin
  trackbar3.Position:=0;
end;

procedure Tmainform.Button14Click(Sender: TObject);
begin
  trackbar4.Position:=0;
end;

procedure Tmainform.TrackBar4Change(Sender: TObject);
var
  valuestr:string;
  value:single;
  reverse:bool;
begin
  // sending ASCII-command
  reverse:=trackbar4.position<0;
  valuestr:=inttostr(abs(trackbar4.Position));
  if length(valuestr)=1 then
    valuestr:='000'+valuestr
  else if length(valuestr)=2 then
    valuestr:='00'+valuestr
  else if length(valuestr)=3 then
    valuestr:='0'+valuestr;

  if reverse then
    SendCommand('Q1-' + valuestr)
  else
    SendCommand('Q1+' + valuestr);

  memo1.Lines.Add(ReceiveAnswer);
end;

end.
