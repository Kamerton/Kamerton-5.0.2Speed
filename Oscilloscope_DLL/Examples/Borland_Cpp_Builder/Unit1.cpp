//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "Unit1.h"
#include "Unit2.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"


#define DllName  "Osc_DLL.dll"

TForm1 *Form1;
//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner) : TForm(Owner)
{
}
//---------------------------------------------------------------------------


Boolean     DllLoaded, Started;
HINSTANCE   DllInst;
int         Scopes[7];

TTstTrd * TstTrd;




char *pEmpty = "";

 // DLL functions
int (__cdecl * ScopeAfterOpenLib)(int Prm);
int (__cdecl * ScopeCreate)      (int Prm ,char  *P_IniName, char *P_IniSuffix);
int (__cdecl * ScopeDestroy)     (int Prm);
int (__cdecl * ScopeShow)        (int Prm);
int (__cdecl * ScopeHide)        (int Prm);
int (__cdecl * ScopeShowNext)    (int Prm, double * ArrDbl);
int (__cdecl * ScopeQuickUpDate) (int Prm);


//---------------------------------------------------------------------------

void __fastcall TForm1::FormCreate(TObject *Sender)
{
  DllLoaded = false;
  Started   = false;
}
//---------------------------------------------------------------------------


void __fastcall TForm1::BtnLoadClick(TObject *Sender)
{
  if ( ! ( DllLoaded) )
  {
    DllInst = NULL;
    DllInst = ::LoadLibrary((char *)DllName);

    if (DllInst==NULL)
    {
     ShowMessage("Error DLL loading 1");
     return;
    }

   ScopeAfterOpenLib = NULL;
   ScopeAfterOpenLib  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "AtOpenLib");
   if (ScopeAfterOpenLib == NULL)
   { FreeLibrary(DllInst); ShowMessage("Error DLL loading 2"); return; }

   ScopeAfterOpenLib (0);

   ScopeShowNext  = NULL;
   ScopeShowNext  = (int (__cdecl  * )(int, double *))GetProcAddress(DllInst, "ShowNext");
   if (ScopeShowNext == NULL)
   { FreeLibrary(DllInst); ShowMessage("Error DLL loading 3"); return; }

   ScopeCreate  = NULL;
   ScopeCreate  = (int (__cdecl  * )(int, char *, char *))GetProcAddress(DllInst, "ScopeCreate");
   if (ScopeCreate == NULL) 
   { FreeLibrary(DllInst); ShowMessage("Error DLL loading 4"); return; }

   ScopeDestroy  = NULL;
   ScopeDestroy  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeDestroy");
   if (ScopeDestroy == NULL) 
   { FreeLibrary(DllInst); ShowMessage("Error DLL loading 5"); return; }

   ScopeShow  = NULL;
   ScopeShow  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeShow");
   if (ScopeShow == NULL)
   { FreeLibrary(DllInst); ShowMessage("Error DLL loading 6"); return; }

   ScopeHide  = NULL;
   ScopeHide  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeHide");
   if (ScopeHide == NULL) 
   { FreeLibrary(DllInst); ShowMessage("Error DLL loading 7"); return; }

   ScopeQuickUpDate  = NULL;
   ScopeQuickUpDate  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "QuickUpDate");
   if (ScopeQuickUpDate == NULL) 
   { FreeLibrary(DllInst); ShowMessage("Error DLL loading 8"); return; }


   // Scopes[1] = ScopeCreate(0, pEmpty, pEmpty);

   Scopes[1] = ScopeCreate(0, NULL, NULL);


   ScopeShow(Scopes[1]);


    BtnLoad->Caption = "Unload library";
    DllLoaded = true;
  }
  else
  {

    FreeLibrary(DllInst);
    ScopeAfterOpenLib = NULL;
    ScopeShowNext     = NULL;
    ScopeCreate       = NULL;
    ScopeQuickUpDate  = NULL;
    ScopeHide         = NULL;
    ScopeShow         = NULL;
    ScopeDestroy      = NULL;

    BtnLoad->Caption = "Load library";
    DllLoaded = false;
  }
} // end of TForm1::BtnLoadClick function.


void __fastcall TForm1::BitBtn1Click(TObject *Sender)
{
int    i;
double D[3], D1[3];

     D[0] = 0 ; D[1] = 0; D[2] = 0;

     D1[0] = 0 ; D1[1] = 0; D1[2] = 0;

 while (Started)
 {

  if (!(DllLoaded))
  { Started = False; BitBtn2->Caption = "Run"; BtnLoad->Enabled = True; return; }


    D[0] = D[0]+11;      if (D[0] >= 280) D[0] = -20;
    D[1] = D[1]+3.1333;  if (D[1] >= 280) D[1] = -20;
    D[2] = D[2]+5.1333;  if (D[2] >= 280) D[2] = -20;
    ScopeShowNext(Scopes[1], D);

   // D1[0] = D1[0]-11;      if (D1[0] < -280) D1[0] = 20;
   // D1[1] = D1[1]-3.1333;  if (D1[1] < -280) D1[1] = 20;
   // D1[2] = D1[2]-5.1333;  if (D1[2] < -280) D1[2] = 20;
   // ScopeShowNext(Scopes[1], D);

   // Application->ProcessMessages;

 }

 ScopeQuickUpDate(Scopes[1]);
// ScopeQuickUpDate(Scopes[2]);

 
}




//---------------------------------------------------------------------------

void __fastcall TForm1::BitBtn2Click(TObject *Sender)
{

 if (!(DllLoaded)) return;
 
 if (!(Started))
 {
   Started = true;

   //TstTrd

  // TstTrd = TTstTrd.Create(True);

 TstTrd = new TTstTrd(true);
 TstTrd->Resume();


   BtnLoad->Enabled = False;
   BitBtn2->Caption = "Stop";
 }
 else
 {
   Started = false;
   BitBtn2->Caption = "Run";
   BtnLoad->Enabled = True;
 }
        
}
//---------------------------------------------------------------------------

void __fastcall TForm1::FormDestroy(TObject *Sender)
{
    Started = false;

    if (DllLoaded)
    {
    FreeLibrary(DllInst);
    ScopeAfterOpenLib = NULL;
    ScopeShowNext     = NULL;
    ScopeCreate       = NULL;
    ScopeQuickUpDate  = NULL;
    ScopeHide         = NULL;
    ScopeShow         = NULL;
    ScopeDestroy      = NULL;
    }
}
//---------------------------------------------------------------------------

