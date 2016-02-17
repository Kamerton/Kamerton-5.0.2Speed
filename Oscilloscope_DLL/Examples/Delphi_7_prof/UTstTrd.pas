unit UTstTrd;

interface

uses
  Classes;

type
  TTstTrd = class(TThread)
  private
    { Private declarations }
  protected
    procedure Execute; override;
  end;

 Var
  TstTrd : TTstTrd;

implementation
 Uses MainUnit;

{ Important: Methods and properties of objects in visual components can only be
  used in a method called using Synchronize, for example,

      Synchronize(UpdateCaption);

  and UpdateCaption could look like,

    procedure TTstTrd.UpdateCaption;
    begin
      Form1.Caption := 'Updated in a thread';
    end; }

{ TTstTrd }

procedure TTstTrd.Execute;
begin
  { Place thread code here }
  // FreeOnTerminate
  MainForm.BitBtn1Click(Nil);
end;

end.
