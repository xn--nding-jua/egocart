object mainform: Tmainform
  Left = 1787
  Top = 169
  Width = 1161
  Height = 648
  Caption = 'eGoCart'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object Label51: TLabel
    Left = 848
    Top = 16
    Width = 92
    Height = 13
    Caption = 'eGoCart Demo-GUI'
  end
  object Label52: TLabel
    Left = 848
    Top = 32
    Width = 223
    Height = 13
    Caption = 'Infos: https://github.com/xn--nding-jua/egocart'
  end
  object Label53: TLabel
    Left = 848
    Top = 80
    Width = 147
    Height = 13
    Caption = 'Response from Motorcontroller:'
  end
  object Button1: TButton
    Left = 16
    Top = 16
    Width = 75
    Height = 25
    Caption = 'Connect'
    TabOrder = 0
    OnClick = Button1Click
  end
  object Memo1: TMemo
    Left = 848
    Top = 96
    Width = 289
    Height = 505
    TabOrder = 1
  end
  object Button3: TButton
    Left = 16
    Top = 56
    Width = 75
    Height = 25
    Caption = 'EN Individuell'
    TabOrder = 2
    OnClick = Butto3Click
  end
  object Button4: TButton
    Left = 176
    Top = 56
    Width = 75
    Height = 25
    Caption = 'Disable'
    TabOrder = 3
    OnClick = Button4Click
  end
  object GroupBox1: TGroupBox
    Left = 16
    Top = 504
    Width = 409
    Height = 97
    Caption = ' Status Motor #1 '
    TabOrder = 4
    object Label1: TLabel
      Left = 16
      Top = 24
      Width = 32
      Height = 13
      Caption = 'Label1'
    end
    object Label2: TLabel
      Left = 16
      Top = 40
      Width = 32
      Height = 13
      Caption = 'Label2'
    end
    object Label3: TLabel
      Left = 176
      Top = 24
      Width = 32
      Height = 13
      Caption = 'Label3'
    end
    object Label4: TLabel
      Left = 176
      Top = 40
      Width = 32
      Height = 13
      Caption = 'Label4'
    end
    object Label5: TLabel
      Left = 176
      Top = 56
      Width = 32
      Height = 13
      Caption = 'Label5'
    end
    object Label6: TLabel
      Left = 176
      Top = 72
      Width = 32
      Height = 13
      Caption = 'Label6'
    end
  end
  object GroupBox2: TGroupBox
    Left = 432
    Top = 504
    Width = 409
    Height = 97
    Caption = ' Status Motor #2 '
    TabOrder = 5
    object Label24: TLabel
      Left = 16
      Top = 24
      Width = 32
      Height = 13
      Caption = 'Label1'
    end
    object Label25: TLabel
      Left = 16
      Top = 40
      Width = 32
      Height = 13
      Caption = 'Label2'
    end
    object Label26: TLabel
      Left = 176
      Top = 24
      Width = 32
      Height = 13
      Caption = 'Label3'
    end
    object Label27: TLabel
      Left = 176
      Top = 40
      Width = 32
      Height = 13
      Caption = 'Label4'
    end
    object Label28: TLabel
      Left = 176
      Top = 56
      Width = 32
      Height = 13
      Caption = 'Label5'
    end
    object Label29: TLabel
      Left = 176
      Top = 72
      Width = 32
      Height = 13
      Caption = 'Label6'
    end
  end
  object Button8: TButton
    Left = 96
    Top = 56
    Width = 75
    Height = 25
    Caption = 'EN Synchron'
    TabOrder = 6
    OnClick = Button8Click
  end
  object GroupBox3: TGroupBox
    Left = 16
    Top = 96
    Width = 409
    Height = 273
    Caption = ' Motor 1 - Speed-Controlled'
    TabOrder = 7
    object Label12: TLabel
      Left = 8
      Top = 96
      Width = 44
      Height = 13
      Caption = '-3500rpm'
    end
    object Label13: TLabel
      Left = 168
      Top = 96
      Width = 29
      Height = 13
      Caption = 'STOP'
    end
    object Label14: TLabel
      Left = 344
      Top = 96
      Width = 41
      Height = 13
      Caption = '3500rpm'
    end
    object Label15: TLabel
      Left = 80
      Top = 96
      Width = 44
      Height = 13
      Caption = '-1750rpm'
    end
    object Label16: TLabel
      Left = 264
      Top = 96
      Width = 41
      Height = 13
      Caption = '1750rpm'
    end
    object Label17: TLabel
      Left = 16
      Top = 24
      Width = 127
      Height = 13
      Caption = 'SetPoint for speed (in rpm):'
    end
    object Label18: TLabel
      Left = 8
      Top = 200
      Width = 33
      Height = 13
      Caption = '0rpm/s'
    end
    object Label19: TLabel
      Left = 168
      Top = 200
      Width = 45
      Height = 13
      Caption = '750rpm/s'
    end
    object Label20: TLabel
      Left = 344
      Top = 200
      Width = 51
      Height = 13
      Caption = '1500rpm/s'
    end
    object Label21: TLabel
      Left = 80
      Top = 200
      Width = 45
      Height = 13
      Caption = '375rpm/s'
    end
    object Label22: TLabel
      Left = 264
      Top = 200
      Width = 51
      Height = 13
      Caption = '1125rpm/s'
    end
    object Label23: TLabel
      Left = 16
      Top = 128
      Width = 166
      Height = 13
      Caption = 'SetPoint for acceleration (in rpm/s):'
    end
    object speedlbl: TLabel
      Left = 24
      Top = 232
      Width = 18
      Height = 23
      Caption = '...'
      Font.Charset = ANSI_CHARSET
      Font.Color = clWindowText
      Font.Height = -19
      Font.Name = 'Tahoma'
      Font.Style = [fsBold]
      ParentFont = False
    end
    object acclbl: TLabel
      Left = 208
      Top = 232
      Width = 18
      Height = 23
      Caption = '...'
      Font.Charset = ANSI_CHARSET
      Font.Color = clWindowText
      Font.Height = -19
      Font.Name = 'Tahoma'
      Font.Style = [fsBold]
      ParentFont = False
    end
    object speedbar: TTrackBar
      Left = 8
      Top = 48
      Width = 369
      Height = 45
      Max = 3500
      Min = -3500
      Frequency = 250
      TabOrder = 0
      TickMarks = tmBoth
      OnChange = speedbarChange
    end
    object accbar: TTrackBar
      Left = 8
      Top = 152
      Width = 369
      Height = 45
      Max = 1500
      Frequency = 100
      TabOrder = 1
      TickMarks = tmBoth
      OnChange = accbarChange
    end
    object Button2: TButton
      Left = 152
      Top = 16
      Width = 75
      Height = 25
      Caption = 'STOP'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clRed
      Font.Height = -11
      Font.Name = 'MS Sans Serif'
      Font.Style = [fsBold]
      ParentFont = False
      TabOrder = 2
      OnClick = Button2Click
    end
    object Button5: TButton
      Left = 296
      Top = 120
      Width = 75
      Height = 25
      Caption = '500rpm/s'
      TabOrder = 3
      OnClick = Button5Click
    end
    object Button9: TButton
      Left = 264
      Top = 16
      Width = 131
      Height = 25
      Caption = 'Enable Speed-Controller'
      TabOrder = 4
      OnClick = Button9Click
    end
  end
  object GroupBox4: TGroupBox
    Left = 432
    Top = 96
    Width = 409
    Height = 273
    Caption = ' Motor 2 - Speed-Controlled '
    TabOrder = 8
    object Label7: TLabel
      Left = 8
      Top = 96
      Width = 44
      Height = 13
      Caption = '-3500rpm'
    end
    object Label8: TLabel
      Left = 168
      Top = 96
      Width = 29
      Height = 13
      Caption = 'STOP'
    end
    object Label9: TLabel
      Left = 344
      Top = 96
      Width = 41
      Height = 13
      Caption = '3500rpm'
    end
    object Label10: TLabel
      Left = 80
      Top = 96
      Width = 44
      Height = 13
      Caption = '-1750rpm'
    end
    object Label11: TLabel
      Left = 264
      Top = 96
      Width = 41
      Height = 13
      Caption = '1750rpm'
    end
    object Label30: TLabel
      Left = 16
      Top = 24
      Width = 127
      Height = 13
      Caption = 'SetPoint for speed (in rpm):'
    end
    object Label31: TLabel
      Left = 8
      Top = 200
      Width = 33
      Height = 13
      Caption = '0rpm/s'
    end
    object Label32: TLabel
      Left = 168
      Top = 200
      Width = 45
      Height = 13
      Caption = '750rpm/s'
    end
    object Label33: TLabel
      Left = 344
      Top = 200
      Width = 51
      Height = 13
      Caption = '1500rpm/s'
    end
    object Label34: TLabel
      Left = 80
      Top = 200
      Width = 45
      Height = 13
      Caption = '375rpm/s'
    end
    object Label35: TLabel
      Left = 264
      Top = 200
      Width = 51
      Height = 13
      Caption = '1125rpm/s'
    end
    object Label36: TLabel
      Left = 16
      Top = 128
      Width = 166
      Height = 13
      Caption = 'SetPoint for acceleration (in rpm/s):'
    end
    object Label37: TLabel
      Left = 16
      Top = 232
      Width = 18
      Height = 23
      Caption = '...'
      Font.Charset = ANSI_CHARSET
      Font.Color = clWindowText
      Font.Height = -19
      Font.Name = 'Tahoma'
      Font.Style = [fsBold]
      ParentFont = False
    end
    object Label38: TLabel
      Left = 192
      Top = 232
      Width = 18
      Height = 23
      Caption = '...'
      Font.Charset = ANSI_CHARSET
      Font.Color = clWindowText
      Font.Height = -19
      Font.Name = 'Tahoma'
      Font.Style = [fsBold]
      ParentFont = False
    end
    object TrackBar1: TTrackBar
      Left = 8
      Top = 48
      Width = 369
      Height = 45
      Max = 3500
      Min = -3500
      Frequency = 250
      TabOrder = 0
      TickMarks = tmBoth
      OnChange = TrackBar1Change
    end
    object TrackBar2: TTrackBar
      Left = 8
      Top = 152
      Width = 369
      Height = 45
      Max = 1500
      Frequency = 100
      TabOrder = 1
      TickMarks = tmBoth
      OnChange = TrackBar2Change
    end
    object Button6: TButton
      Left = 152
      Top = 16
      Width = 75
      Height = 25
      Caption = 'STOP'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clRed
      Font.Height = -11
      Font.Name = 'MS Sans Serif'
      Font.Style = [fsBold]
      ParentFont = False
      TabOrder = 2
      OnClick = Button6Click
    end
    object Button7: TButton
      Left = 296
      Top = 120
      Width = 75
      Height = 25
      Caption = '500rpm/s'
      TabOrder = 3
      OnClick = Button7Click
    end
    object Button10: TButton
      Left = 264
      Top = 16
      Width = 131
      Height = 25
      Caption = 'Enable Speed-Controller'
      TabOrder = 4
      OnClick = Button10Click
    end
  end
  object GroupBox5: TGroupBox
    Left = 16
    Top = 376
    Width = 409
    Height = 121
    Caption = ' Motor 1 - Current-Controlled '
    TabOrder = 9
    object Label39: TLabel
      Left = 8
      Top = 96
      Width = 29
      Height = 13
      Caption = '-100%'
    end
    object Label40: TLabel
      Left = 168
      Top = 96
      Width = 29
      Height = 13
      Caption = 'STOP'
    end
    object Label41: TLabel
      Left = 344
      Top = 96
      Width = 32
      Height = 13
      Caption = '+100%'
    end
    object Label42: TLabel
      Left = 80
      Top = 96
      Width = 23
      Height = 13
      Caption = '-50%'
    end
    object Label43: TLabel
      Left = 264
      Top = 96
      Width = 26
      Height = 13
      Caption = '+50%'
    end
    object Label44: TLabel
      Left = 16
      Top = 24
      Width = 122
      Height = 13
      Caption = 'SetPoint for current (in %):'
    end
    object Button11: TButton
      Left = 264
      Top = 16
      Width = 131
      Height = 25
      Caption = 'Enable Current-Controller'
      TabOrder = 0
      OnClick = Button11Click
    end
    object TrackBar3: TTrackBar
      Left = 8
      Top = 48
      Width = 369
      Height = 45
      Max = 9999
      Min = -9999
      Frequency = 250
      TabOrder = 1
      TickMarks = tmBoth
      OnChange = TrackBar3Change
    end
    object Button13: TButton
      Left = 152
      Top = 16
      Width = 75
      Height = 25
      Caption = 'STOP'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clRed
      Font.Height = -11
      Font.Name = 'MS Sans Serif'
      Font.Style = [fsBold]
      ParentFont = False
      TabOrder = 2
      OnClick = Button13Click
    end
  end
  object GroupBox6: TGroupBox
    Left = 432
    Top = 376
    Width = 409
    Height = 121
    Caption = ' Motor 2 - Current-Controlled '
    TabOrder = 10
    object Label45: TLabel
      Left = 8
      Top = 96
      Width = 29
      Height = 13
      Caption = '-100%'
    end
    object Label46: TLabel
      Left = 168
      Top = 96
      Width = 29
      Height = 13
      Caption = 'STOP'
    end
    object Label47: TLabel
      Left = 344
      Top = 96
      Width = 32
      Height = 13
      Caption = '+100%'
    end
    object Label48: TLabel
      Left = 80
      Top = 96
      Width = 23
      Height = 13
      Caption = '-50%'
    end
    object Label49: TLabel
      Left = 264
      Top = 96
      Width = 26
      Height = 13
      Caption = '+50%'
    end
    object Label50: TLabel
      Left = 16
      Top = 24
      Width = 122
      Height = 13
      Caption = 'SetPoint for current (in %):'
    end
    object Button12: TButton
      Left = 264
      Top = 16
      Width = 131
      Height = 25
      Caption = 'Enable Current-Controller'
      TabOrder = 0
      OnClick = Button12Click
    end
    object TrackBar4: TTrackBar
      Left = 8
      Top = 48
      Width = 369
      Height = 45
      Max = 9999
      Min = -9999
      Frequency = 250
      TabOrder = 1
      TickMarks = tmBoth
      OnChange = TrackBar4Change
    end
    object Button14: TButton
      Left = 152
      Top = 16
      Width = 75
      Height = 25
      Caption = 'STOP'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clRed
      Font.Height = -11
      Font.Name = 'MS Sans Serif'
      Font.Style = [fsBold]
      ParentFont = False
      TabOrder = 2
      OnClick = Button14Click
    end
  end
  object XPManifest1: TXPManifest
    Left = 216
    Top = 16
  end
  object comport: TCommPortDriver
    Port = pnCustom
    PortName = '\\.\COM9'
    BaudRate = brCustom
    BaudRateValue = 38400
    Left = 248
    Top = 16
  end
  object Timer1: TTimer
    Enabled = False
    Interval = 500
    OnTimer = Timer1Timer
    Left = 320
    Top = 16
  end
end
