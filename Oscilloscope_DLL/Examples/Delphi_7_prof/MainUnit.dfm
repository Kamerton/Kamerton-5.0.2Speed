object MainForm: TMainForm
  Left = -2
  Top = -2
  Width = 1110
  Height = 129
  AutoSize = True
  BorderStyle = bsSizeToolWin
  Caption = 'MainForm'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  FormStyle = fsStayOnTop
  OldCreateOrder = False
  OnCreate = FormCreate
  OnDestroy = FormDestroy
  PixelsPerInch = 99
  TextHeight = 13
  object Label1: TLabel
    Left = 545
    Top = 4
    Width = 44
    Height = 13
    Caption = 'Form top:'
  end
  object Label2: TLabel
    Left = 545
    Top = 28
    Width = 43
    Height = 13
    Caption = 'Form left:'
  end
  object Label3: TLabel
    Left = 769
    Top = 4
    Width = 58
    Height = 13
    Caption = 'Form height:'
  end
  object Label4: TLabel
    Left = 769
    Top = 28
    Width = 54
    Height = 13
    Caption = 'Form width:'
  end
  object Label5: TLabel
    Left = 1
    Top = 84
    Width = 81
    Height = 13
    Caption = 'Controlling beam:'
  end
  object Label6: TLabel
    Left = 193
    Top = 84
    Width = 66
    Height = 13
    Caption = 'Vertical scale:'
  end
  object Label7: TLabel
    Left = 417
    Top = 84
    Width = 67
    Height = 13
    Caption = 'Vertical offset:'
  end
  object BtnLoad: TBitBtn
    Left = 0
    Top = 6
    Width = 79
    Height = 20
    Caption = 'Load Scope'
    TabOrder = 0
    OnClick = BtnLoadClick
  end
  object BitBtn2: TBitBtn
    Left = 244
    Top = 6
    Width = 49
    Height = 20
    Caption = 'Run'
    TabOrder = 1
    OnClick = BitBtn2Click
  end
  object CheckSlow: TCheckBox
    Left = 88
    Top = 9
    Width = 56
    Height = 14
    Caption = 'Slowly'
    TabOrder = 2
    OnClick = CheckSlowClick
  end
  object BtnClean: TBitBtn
    Left = 153
    Top = 6
    Width = 79
    Height = 20
    Caption = 'Clean scope'
    TabOrder = 3
    OnClick = BtnCleanClick
  end
  object BtnShowPanel: TBitBtn
    Left = 301
    Top = 4
    Width = 75
    Height = 25
    Caption = 'Show panel'
    TabOrder = 4
    OnClick = BtnShowPanelClick
  end
  object BtnHidePanel: TBitBtn
    Left = 381
    Top = 4
    Width = 75
    Height = 25
    Caption = 'Hide panel'
    TabOrder = 5
    OnClick = BtnHidePanelClick
  end
  object EditTop: TEdit
    Left = 593
    Top = 0
    Width = 69
    Height = 21
    TabOrder = 6
    Text = '100'
  end
  object BtnPos: TBitBtn
    Left = 461
    Top = 4
    Width = 75
    Height = 25
    Caption = 'Set position'
    TabOrder = 7
    OnClick = BtnPosClick
  end
  object EditLeft: TEdit
    Left = 593
    Top = 24
    Width = 69
    Height = 21
    TabOrder = 8
    Text = '200'
  end
  object BtnSize: TBitBtn
    Left = 685
    Top = 4
    Width = 75
    Height = 25
    Caption = 'Set size'
    TabOrder = 9
    OnClick = BtnSizeClick
  end
  object EditWidth: TEdit
    Left = 829
    Top = 24
    Width = 57
    Height = 21
    TabOrder = 10
    Text = '500'
  end
  object EditHeight: TEdit
    Left = 829
    Top = 0
    Width = 57
    Height = 21
    TabOrder = 11
    Text = '400'
  end
  object BtnGridSize: TBitBtn
    Left = 913
    Top = 0
    Width = 125
    Height = 25
    Caption = 'Set cell size in pixels:'
    TabOrder = 12
    OnClick = BtnGridSizeClick
  end
  object EditCellSize: TEdit
    Left = 1045
    Top = 4
    Width = 57
    Height = 21
    TabOrder = 13
    Text = '20'
  end
  object BtnGridSamples: TBitBtn
    Left = 913
    Top = 28
    Width = 125
    Height = 25
    Caption = 'Set cell size in samples:'
    TabOrder = 14
    OnClick = BtnGridSamplesClick
  end
  object EditCellSmpls: TEdit
    Left = 1045
    Top = 32
    Width = 57
    Height = 21
    TabOrder = 15
    Text = '20'
  end
  object BtnNewCap: TBitBtn
    Left = 1
    Top = 32
    Width = 97
    Height = 25
    Caption = 'Set new caption:'
    TabOrder = 16
    OnClick = BtnNewCapClick
  end
  object EditNewCap: TEdit
    Left = 109
    Top = 36
    Width = 428
    Height = 21
    TabOrder = 17
    Text = 'Oscilloscope'
  end
  object CmbBxBeamCtrl: TComboBox
    Left = 89
    Top = 80
    Width = 93
    Height = 21
    Style = csDropDownList
    ItemHeight = 13
    ItemIndex = 0
    TabOrder = 18
    Text = 'First beam'
    Items.Strings = (
      'First beam'
      'Second beam'
      'Third beam')
  end
  object EditVertScale: TEdit
    Left = 265
    Top = 80
    Width = 69
    Height = 21
    TabOrder = 19
    Text = '20'
  end
  object BtnVScaleSet: TBitBtn
    Left = 337
    Top = 80
    Width = 53
    Height = 21
    Caption = 'Set'
    TabOrder = 20
    OnClick = BtnVScaleSetClick
  end
  object EditVertOffs: TEdit
    Left = 489
    Top = 80
    Width = 69
    Height = 21
    TabOrder = 21
    Text = '0'
  end
  object BtnVoffsSet: TBitBtn
    Left = 565
    Top = 80
    Width = 53
    Height = 21
    Caption = 'Set'
    TabOrder = 22
    OnClick = BtnVoffsSetClick
  end
end
