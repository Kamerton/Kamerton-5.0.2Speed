unit MainUnit;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls, Buttons, UTstTrd, ExtCtrls;


 Type
 TArrDbl  = Array [0..2] of Double;
 PTArrDbl = ^TArrDbl;

type
  TMainForm = class(TForm)
    BtnLoad: TBitBtn;
    BitBtn2: TBitBtn;
    CheckSlow: TCheckBox;
    BtnClean: TBitBtn;
    BtnShowPanel: TBitBtn;
    BtnHidePanel: TBitBtn;
    EditTop: TEdit;
    BtnPos: TBitBtn;
    Label1: TLabel;
    Label2: TLabel;
    EditLeft: TEdit;
    BtnSize: TBitBtn;
    Label3: TLabel;
    Label4: TLabel;
    EditWidth: TEdit;
    EditHeight: TEdit;
    BtnGridSize: TBitBtn;
    EditCellSize: TEdit;
    BtnGridSamples: TBitBtn;
    EditCellSmpls: TEdit;
    BtnNewCap: TBitBtn;
    EditNewCap: TEdit;
    Label5: TLabel;
    CmbBxBeamCtrl: TComboBox;
    Label6: TLabel;
    EditVertScale: TEdit;
    BtnVScaleSet: TBitBtn;
    Label7: TLabel;
    EditVertOffs: TEdit;
    BtnVoffsSet: TBitBtn;
    procedure BtnLoadClick(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure BitBtn1Click(Sender: TObject);
    procedure BitBtn2Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure CheckSlowClick(Sender: TObject);
    procedure BtnCleanClick(Sender: TObject);
    procedure FormActivate(Sender: TObject);
    procedure BtnShowPanelClick(Sender: TObject);
    procedure BtnHidePanelClick(Sender: TObject);
    procedure BtnPosClick(Sender: TObject);
    procedure BtnSizeClick(Sender: TObject);
    procedure BtnGridSizeClick(Sender: TObject);
    procedure BtnGridSamplesClick(Sender: TObject);
    procedure BtnNewCapClick(Sender: TObject);
    procedure BtnVScaleSetClick(Sender: TObject);
    procedure BtnVoffsSetClick(Sender: TObject);
  private
    { Private declarations }
  public
    { Public declarations }

    DllInst     : Longint; // DLL handle.

    // DLL functions begin
     ScopeAfterOpenLib  : function (Prm : Integer)  : Integer; cdecl;
     ScopeCreate        : function (Prm : Integer; P_IniName : Pointer; P_IniSuffix : Pointer) : Integer; cdecl;
     ScopeDestroy       : function (Prm : Integer) : Integer; cdecl;
     ScopeShow          : function (Prm : Integer) : Integer; cdecl;
     ScopeHide          : function (Prm : Integer) : Integer; cdecl;
     ScopeCleanBuffers  : function (ScopeHandle : Integer) : Integer; cdecl;
     ScopeShowNext      : function (Prm : Integer; PrmD : PTArrDbl) : Integer; cdecl;
     ScopeQuickUpDate   : function (Prm : Integer) : Integer; cdecl;
     ScopeExtNext       : function (ScopeHandle : Integer; PDbl : Pointer) : Integer; cdecl;
     
     ScopeSetPanelState     : function (Prm : Integer; VisState : Integer) : Integer; cdecl;
     ScopeSetFormPos        : function (Prm : Integer; FormLeft : Integer; FormTop : Integer) : Integer; cdecl;
     ScopeSetFormSize       : function (Prm : Integer; FormWidth : Integer; FormHeight : Integer) : Integer; cdecl;
     ScopeSetCellPixelSize  : function (Prm : Integer; CellPixelSize : Integer) : Integer; cdecl;
     ScopeSetCellSampleSize : function (Prm : Integer; CellSampleSize : Double) : Integer; cdecl;
     ScopeSetCaption        : function (Prm : Integer; P_ZeroTermStr : Pointer) : Integer; cdecl;
     ScopeSetAmpScale       : function (Prm : Integer; BeamNum : Integer; AmpScale : Double) : Integer; cdecl;
     ScopeSetAmpOffset      : function (Prm : Integer; BeamNum : Integer; AmpOffset : Double) : Integer; cdecl;

    // DLL functions end
  end;

var
  MainForm: TMainForm;

Const
 DllName : String = 'Osc_DLL.dll' + Char(0);

implementation

{$R *.dfm}

Var
 Scopes : Array [1..8] of integer;
 DllLoaded : Boolean;
 Started   : Boolean;
 Slowly    : Boolean;

procedure TMainForm.BtnLoadClick(Sender: TObject);
Var
 Rslt : Integer;
 S, S1 : String;

begin
 Scopes[2] := 0; 

 If (BtnLoad.Caption = 'Load Scope') then
 begin
  ScopeAfterOpenLib := NIL;
  DllInst := LoadLibrary(PChar(DllName));
   if (DllInst = 0) then  begin ShowMessage('Error DLL loading 1'); Exit; end;
   
   ScopeAfterOpenLib  := GetProcAddress(DllInst, 'AtOpenLib');
   if (@ScopeAfterOpenLib = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 2'); Exit; end;


   ScopeAfterOpenLib(0);

   
   ScopeShowNext := Nil;
   ScopeShowNext := GetProcAddress(DllInst, 'ShowNext');
   if (@ScopeShowNext = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 3'); Exit; end;

   ScopeCreate := Nil;
   ScopeCreate := GetProcAddress(DllInst, 'ScopeCreate');
   if (@ScopeCreate = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 4'); Exit; end;

   ScopeDestroy := Nil;
   ScopeDestroy := GetProcAddress(DllInst, 'ScopeDestroy');
   if (@ScopeDestroy = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 5'); Exit; end;

   ScopeShow := Nil;
   ScopeShow := GetProcAddress(DllInst, 'ScopeShow');
   if (@ScopeShow = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 6'); Exit; end;

   ScopeHide := Nil;
   ScopeHide := GetProcAddress(DllInst, 'ScopeHide');
   if (@ScopeHide = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 7'); Exit; end;


   ScopeCleanBuffers := Nil;
   ScopeCleanBuffers := GetProcAddress(DllInst, 'ScopeCleanBuffers');
   if (@ScopeCleanBuffers = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 8');  end;


   ScopeQuickUpDate := Nil;
   ScopeQuickUpDate := GetProcAddress(DllInst, 'QuickUpDate');
   if (@ScopeQuickUpDate = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 9'); Exit; end;


   ScopeExtNext := Nil;
   ScopeExtNext := GetProcAddress(DllInst, 'ExternalNext');
   if (@ScopeExtNext = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 10'); Exit; end;


   ScopeSetPanelState := Nil;
   ScopeSetPanelState := GetProcAddress(DllInst, 'ScopeSetPanelState');
   if (@ScopeSetPanelState = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 11'); Exit; end;


   ScopeSetFormPos := Nil;
   ScopeSetFormPos := GetProcAddress(DllInst, 'ScopeSetFormPos');
   if (@ScopeSetFormPos = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 12'); Exit; end;


   ScopeSetFormSize := Nil;
   ScopeSetFormSize := GetProcAddress(DllInst, 'ScopeSetFormSize');
   if (@ScopeSetFormSize = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 13'); Exit; end;


   ScopeSetCellPixelSize := Nil;
   ScopeSetCellPixelSize := GetProcAddress(DllInst, 'ScopeSetCellPixelSize');
   if (@ScopeSetCellPixelSize = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 14'); Exit; end;


   ScopeSetCellSampleSize := Nil;
   ScopeSetCellSampleSize := GetProcAddress(DllInst, 'ScopeSetCellSampleSize');
   if (@ScopeSetCellSampleSize = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 15'); Exit; end;


   ScopeSetCaption := Nil;
   ScopeSetCaption := GetProcAddress(DllInst, 'ScopeSetCaption');
   if (@ScopeSetCaption = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 16'); Exit; end;

   
   ScopeSetAmpScale := Nil;
   ScopeSetAmpScale := GetProcAddress(DllInst, 'ScopeSetAmpScale');
   if (@ScopeSetAmpScale = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 17'); Exit; end;


   ScopeSetAmpOffset := Nil;
   ScopeSetAmpOffset := GetProcAddress(DllInst, 'ScopeSetAmpOffset');
   if (@ScopeSetAmpOffset = Nil) then
   begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 18'); Exit; end;


   // S := 'Scope_Desk_1.ini';   S1 := '22' + Char(0);
   //
   // Scopes[1] := ScopeCreate(0, @(S[1]), @(S1[1]));
   // ScopeShow(Scopes[1]);


  //  S := 'C:\Tmichael\Scope_Desk_1.ini' + Char(0);

  //  S := 'Scope_DeskM.ini' + Char(0);
    
    Scopes[2] := ScopeCreate(0, {@(S[1])}Nil, Nil);

   // ShowMessage (IntToStr(Scopes[2]));

    ScopeShow(Scopes[2]);

  //  Scopes[3] := ScopeCreate(0);
  //  ScopeShow(Scopes[3]);

   // if (Rslt <> 0) then
   // begin FreeLibrary(DllInst); ShowMessage('Error DLL loading 8'); Exit; end;

   BtnLoad.Caption := 'Unoad Scope';
   DllLoaded := True;
 end
 else
 begin

  DllLoaded := False;

  ScopeDestroy(Scopes[1]);
  ScopeDestroy(Scopes[2]);

  FreeLibrary(DllInst);

  ScopeAfterOpenLib      := NIL;
  ScopeShowNext          := Nil;
  ScopeCreate            := Nil;
  ScopeQuickUpDate       := Nil;
  ScopeHide              := Nil;
  ScopeCleanBuffers      := Nil;
  ScopeShow              := Nil;
  ScopeDestroy           := Nil;
  ScopeExtNext           := Nil;
  ScopeSetPanelState     := Nil;
  ScopeSetFormPos        := Nil;
  ScopeSetFormSize       := Nil;
  ScopeSetCellPixelSize  := Nil;
  ScopeSetCellSampleSize := Nil;
  ScopeSetCaption        := Nil;
  ScopeSetAmpScale       := Nil;
  ScopeSetAmpOffset      := Nil;

  BtnLoad.Caption := 'Load Scope';
 end;

end; {of TMainForm.BtnLoadClick}



procedure TMainForm.FormDestroy(Sender: TObject);
begin
//  ScopeDestroy(Scopes[1]);

//  ScopeDestroy(Scopes[2]);

  Started := False; Sleep(100);

  FreeLibrary(DllInst);
  
end;

procedure TMainForm.BitBtn1Click(Sender: TObject);
Var
 D, D1     : TArrDbl;
 i         : Integer;
 Cnt       : Integer;
 Db        : Double;
begin

     D[0] := 0 ; D[1] := 0; D[2] := 0;

     D1[0] := 0 ; D1[1] := 0; D1[2] := 0;

 While Started do
 begin


 For Cnt := 0 to 100 do begin

  If (Not(DllLoaded)) then
  begin Started := False; BitBtn2.Caption := 'Run'; BtnLoad.Enabled := True; Exit; end;


  //  D[0] := D[0]+11;      if D[0] >= 280 then D[0] := -20;
  //  D[1] := D[1]+3.1333;  if D[1] >= 280 then D[1] := -20;
  //  D[2] := D[2]+5.1333;  if D[2] >= 280 then D[2] := -20;
  //  ScopeShowNext(Scopes[1], @D);

    D1[0] := D1[0]-11;      if D1[0] < -280 then D1[0] := 20;
    D1[1] := D1[1]-3.1333;  if D1[1] < -280 then D1[1] := 20;
    D1[2] := D1[2]-5.1333;  if D1[2] < -280 then D1[2] := 20;

    Db := D1[0] + D1[1] + + D1[2];

    if Started then begin
      ScopeExtNext(Scopes[2], @Db);
      ScopeShowNext(Scopes[2], @D1);
    end else Exit;

 //  Application.ProcessMessages;

 end; // Cnt

 If Slowly then Sleep(25);

 end;

 //ScopeQuickUpDate(Scopes[1]);
 ScopeQuickUpDate(Scopes[2]);
 Application.ProcessMessages;

end;


{$O-}
procedure TMainForm.BitBtn2Click(Sender: TObject);

begin

// tpIdle          The thread executes only when the system is idle-Windows won't interrupt other threads to execute a thread with tpIdle priority.
// tpLowest        The thread's priority is two points below normal.
// tpLower         The thread's priority is one point below normal.
// tpNormal        The thread has normal priority.
// tpHigher        The thread's priority is one point above normal.
// tpHighest       The thread's priority is two points above normal.
// tpTimeCritical  The thread gets highest priority.

  // TstTrd.Priority := tpTimeCritical;

 if (BitBtn2.Caption = 'Run') then
 begin
  BtnLoad.Enabled := False;
  TstTrd := TTstTrd.Create(True);
 // TstTrd.Priority := tpHighest;	 //tpHigher;
  BitBtn2.Caption := 'Stop';
  Started := True;
  TstTrd.Resume;

 end else
 begin
  Started := False;
  BitBtn2.Caption := 'Run';
  if (Scopes[2] <> 0) then ScopeQuickUpDate(Scopes[2]);
  BtnLoad.Enabled := True;
 end;

end;
{$O+}


procedure TMainForm.FormCreate(Sender: TObject);
 Var
 i : Integer;
begin
 DllLoaded := False;
 Started := False;
 Slowly := False;
 for i := 1 to 8 do Scopes[i] := 0;
 Application.OnActivate := FormActivate;
end;

procedure TMainForm.CheckSlowClick(Sender: TObject);
begin
 Slowly := CheckSlow.Checked;
end;

procedure TMainForm.BtnCleanClick(Sender: TObject);
begin
  ScopeCleanBuffers(Scopes[2]);
end;

procedure TMainForm.FormActivate(Sender: TObject);
begin
 if (Scopes[2] <> 0) then ScopeShow(Scopes[2]);
end;


procedure TMainForm.BtnShowPanelClick(Sender: TObject);
begin
  if (Scopes[2] <> 0) then ScopeSetPanelState(Scopes[2], 1);
end;


procedure TMainForm.BtnHidePanelClick(Sender: TObject);
begin
   if (Scopes[2] <> 0) then ScopeSetPanelState(Scopes[2], 0);
end;

procedure TMainForm.BtnPosClick(Sender: TObject);
Var
 S : String;
 NewLeft, NewTop, i : Integer;
begin
  if (Scopes[2] = 0) then Exit;

  i := 0;
  S := EditLeft.Text;
  Val(S, NewLeft, i);
  if (i <> 0) then begin ShowMessage('Error. Not correct new horizontal position of oscilloscope form.'); Exit; end;

  S := EditTop.Text;
  Val(S, NewTop, i);
  if (i <> 0) then begin ShowMessage('Error. Not correct new vertical position of oscilloscope form.'); Exit; end;

  ScopeSetFormPos(Scopes[2], NewLeft, NewTop);

end; {of BtnPosClick}

procedure TMainForm.BtnSizeClick(Sender: TObject);
Var
 S : String;
 NewWidth, NewHeight, i : Integer;
begin
  if (Scopes[2] = 0) then Exit;

  i := 0;
  S := EditWidth.Text;
  Val(S, NewWidth, i);
  if (i <> 0) then begin ShowMessage('Error. Not correct new width of oscilloscope form.'); Exit; end;

  S := EditHeight.Text;
  Val(S, NewHeight, i);
  if (i <> 0) then begin ShowMessage('Error. Not correct new height of oscilloscope form.'); Exit; end;

  ScopeSetFormSize(Scopes[2], NewWidth, NewHeight);
end;

procedure TMainForm.BtnGridSizeClick(Sender: TObject);
Var
 S : String;
 NewCellSize, i : Integer;
begin
  if (Scopes[2] = 0) then Exit;

  i := 0;
  S := EditCellSize.Text;
  Val(S, NewCellSize, i);
  if (i <> 0) then begin ShowMessage('Error. Not correct new value of cell size in screen pixels.'); Exit; end;

  ScopeSetCellPixelSize(Scopes[2], NewCellSize);

end;



procedure TMainForm.BtnGridSamplesClick(Sender: TObject);
Var
 S : String;
 NewCellSamples : Double;
 i : Integer;
begin
  if (Scopes[2] = 0) then Exit;

  i := 0;
  S := EditCellSmpls.Text;
  Val(S, NewCellSamples, i);

  if (i <> 0) then begin ShowMessage('Error. Not correct new value of cell samples capacity.'); Exit; end;

  ScopeSetCellSampleSize(Scopes[2], NewCellSamples);
end;

procedure TMainForm.BtnNewCapClick(Sender: TObject);
Var
 S : String;
begin
   if (Scopes[2] = 0) then Exit;
   S := '';
   S := EditNewCap.Text + #0;

   ScopeSetCaption(Scopes[2],  @(S[1]) );

end;

procedure TMainForm.BtnVScaleSetClick(Sender: TObject);
Var
  S          : String;
  NewVscale  : Double;
  i          : Integer;
begin
    if (Scopes[2] = 0) then Exit;

    i := 0;
    S := EditVertScale.Text;
    Val(S, NewVscale, i);
    if (i <> 0) then begin ShowMessage('Error. Not correct new value of vertical scale.'); Exit; end;
    ScopeSetAmpScale(Scopes[2], CmbBxBeamCtrl.ItemIndex,  NewVscale);

end;





           
procedure TMainForm.BtnVoffsSetClick(Sender: TObject);
Var
  S          : String;
  NewOffs    : Double;
  i          : Integer;
begin
    if (Scopes[2] = 0) then Exit;

    i := 0;
    S := EditVertOffs.Text;
    Val(S, NewOffs, i);
    if (i <> 0) then begin ShowMessage('Error. Not correct new value of vertical offset.'); Exit; end;
    ScopeSetAmpOffset(Scopes[2], CmbBxBeamCtrl.ItemIndex,  NewOffs);

end;

end.
