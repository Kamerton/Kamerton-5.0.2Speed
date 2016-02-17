program Ssc_DLLtst;

uses
  Forms,
  MainUnit in 'MainUnit.pas' {MainForm},
  UTstTrd in 'UTstTrd.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TMainForm, MainForm);
  Application.Run;
end.
