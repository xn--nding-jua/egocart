object mainform: Tmainform
  Left = 1902
  Top = 128
  Width = 855
  Height = 736
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
  object Label12: TLabel
    Left = 16
    Top = 176
    Width = 44
    Height = 13
    Caption = '-3500rpm'
  end
  object Label13: TLabel
    Left = 176
    Top = 176
    Width = 29
    Height = 13
    Caption = 'STOP'
  end
  object Label14: TLabel
    Left = 352
    Top = 176
    Width = 41
    Height = 13
    Caption = '3500rpm'
  end
  object Label15: TLabel
    Left = 88
    Top = 176
    Width = 44
    Height = 13
    Caption = '-1750rpm'
  end
  object Label16: TLabel
    Left = 272
    Top = 176
    Width = 41
    Height = 13
    Caption = '1750rpm'
  end
  object Label17: TLabel
    Left = 24
    Top = 104
    Width = 127
    Height = 13
    Caption = 'SetPoint for speed (in rpm):'
  end
  object Label18: TLabel
    Left = 16
    Top = 280
    Width = 33
    Height = 13
    Caption = '0rpm/s'
  end
  object Label19: TLabel
    Left = 176
    Top = 280
    Width = 45
    Height = 13
    Caption = '750rpm/s'
  end
  object Label20: TLabel
    Left = 352
    Top = 280
    Width = 51
    Height = 13
    Caption = '1500rpm/s'
  end
  object Label21: TLabel
    Left = 88
    Top = 280
    Width = 45
    Height = 13
    Caption = '375rpm/s'
  end
  object Label22: TLabel
    Left = 272
    Top = 280
    Width = 51
    Height = 13
    Caption = '1125rpm/s'
  end
  object Label23: TLabel
    Left = 24
    Top = 208
    Width = 166
    Height = 13
    Caption = 'SetPoint for acceleration (in rpm/s):'
  end
  object speedlbl: TLabel
    Left = 32
    Top = 312
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
    Left = 216
    Top = 312
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
  object Label7: TLabel
    Left = 424
    Top = 176
    Width = 44
    Height = 13
    Caption = '-3500rpm'
  end
  object Label8: TLabel
    Left = 584
    Top = 176
    Width = 29
    Height = 13
    Caption = 'STOP'
  end
  object Label9: TLabel
    Left = 760
    Top = 176
    Width = 41
    Height = 13
    Caption = '3500rpm'
  end
  object Label10: TLabel
    Left = 496
    Top = 176
    Width = 44
    Height = 13
    Caption = '-1750rpm'
  end
  object Label11: TLabel
    Left = 680
    Top = 176
    Width = 41
    Height = 13
    Caption = '1750rpm'
  end
  object Label30: TLabel
    Left = 432
    Top = 104
    Width = 127
    Height = 13
    Caption = 'SetPoint for speed (in rpm):'
  end
  object Label31: TLabel
    Left = 424
    Top = 280
    Width = 33
    Height = 13
    Caption = '0rpm/s'
  end
  object Label32: TLabel
    Left = 584
    Top = 280
    Width = 45
    Height = 13
    Caption = '750rpm/s'
  end
  object Label33: TLabel
    Left = 760
    Top = 280
    Width = 51
    Height = 13
    Caption = '1500rpm/s'
  end
  object Label34: TLabel
    Left = 496
    Top = 280
    Width = 45
    Height = 13
    Caption = '375rpm/s'
  end
  object Label35: TLabel
    Left = 680
    Top = 280
    Width = 51
    Height = 13
    Caption = '1125rpm/s'
  end
  object Label36: TLabel
    Left = 432
    Top = 208
    Width = 166
    Height = 13
    Caption = 'SetPoint for acceleration (in rpm/s):'
  end
  object Label37: TLabel
    Left = 432
    Top = 312
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
    Left = 608
    Top = 312
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
    Left = 16
    Top = 496
    Width = 313
    Height = 193
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
  object speedbar: TTrackBar
    Left = 16
    Top = 128
    Width = 369
    Height = 45
    Max = 3500
    Min = -3500
    Frequency = 250
    TabOrder = 4
    TickMarks = tmBoth
    OnChange = speedbarChange
  end
  object accbar: TTrackBar
    Left = 16
    Top = 232
    Width = 369
    Height = 45
    Max = 1500
    Frequency = 100
    TabOrder = 5
    TickMarks = tmBoth
    OnChange = accbarChange
  end
  object Button2: TButton
    Left = 304
    Top = 96
    Width = 75
    Height = 25
    Caption = 'STOP'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clRed
    Font.Height = -11
    Font.Name = 'MS Sans Serif'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 6
    OnClick = Button2Click
  end
  object Button5: TButton
    Left = 304
    Top = 200
    Width = 75
    Height = 25
    Caption = '500rpm/s'
    TabOrder = 7
    OnClick = Button5Click
  end
  object GroupBox1: TGroupBox
    Left = 16
    Top = 352
    Width = 401
    Height = 137
    Caption = ' Status Motor #1 '
    TabOrder = 8
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
    Left = 424
    Top = 352
    Width = 401
    Height = 137
    Caption = ' Status Motor #2 '
    TabOrder = 9
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
  object TrackBar1: TTrackBar
    Left = 424
    Top = 128
    Width = 369
    Height = 45
    Max = 3500
    Min = -3500
    Frequency = 250
    TabOrder = 10
    TickMarks = tmBoth
    OnChange = TrackBar1Change
  end
  object TrackBar2: TTrackBar
    Left = 424
    Top = 232
    Width = 369
    Height = 45
    Max = 1500
    Frequency = 100
    TabOrder = 11
    TickMarks = tmBoth
    OnChange = TrackBar2Change
  end
  object Button6: TButton
    Left = 712
    Top = 96
    Width = 75
    Height = 25
    Caption = 'STOP'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clRed
    Font.Height = -11
    Font.Name = 'MS Sans Serif'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 12
    OnClick = Button6Click
  end
  object Button7: TButton
    Left = 712
    Top = 200
    Width = 75
    Height = 25
    Caption = '500rpm/s'
    TabOrder = 13
    OnClick = Button7Click
  end
  object Button8: TButton
    Left = 96
    Top = 56
    Width = 75
    Height = 25
    Caption = 'EN Synchron'
    TabOrder = 14
    OnClick = Button8Click
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
