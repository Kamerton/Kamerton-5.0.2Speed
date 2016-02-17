object Form1: TForm1
  Left = 18
  Top = 216
  Width = 314
  Height = 106
  Caption = 'Test'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -13
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  OnCreate = FormCreate
  OnDestroy = FormDestroy
  PixelsPerInch = 120
  TextHeight = 16
  object BtnLoad: TBitBtn
    Left = 20
    Top = 20
    Width = 125
    Height = 33
    Caption = 'Load library'
    TabOrder = 0
    OnClick = BtnLoadClick
  end
  object BitBtn2: TBitBtn
    Left = 184
    Top = 20
    Width = 101
    Height = 33
    Caption = 'Run'
    TabOrder = 1
    OnClick = BitBtn2Click
  end
end
