// VC_TSTDlg.cpp : implementation file
//

#include "stdafx.h"
#include "VC_TST.h"
#include "VC_TSTDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#include "OSC_DLL.H"

extern  char *pEmpty = "";
    // DLL functions begin
int (__cdecl  * ScopeAfterOpenLib)(int Prm);
int (__cdecl  *     ScopeCreate)   (int Prm ,char  *P_IniName, char *P_IniSuffix);
int (__cdecl  *     ScopeDestroy)  (int Prm);
int (__cdecl  *     ScopeShow)      (int Prm);
int (__cdecl  *     ScopeHide)       (int Prm);
int (__cdecl  *     ScopeShowNext)    (int Prm, struct  tagTArrDbl *PrmD );
int (__cdecl  *     ScopeQuickUpDate)  (int Prm);

int fLoaded;
HINSTANCE DllInst;
int fRun;
int fThreadFinished;

int Scopes[7];

/*
     ScopeAfterOpenLib  : function (Prm : Integer)  : Integer; cdecl;
     ScopeCreate        : function (Prm : Integer; P_IniName : Pointer; P_IniSuffix : Pointer) : Integer; cdecl;
     ScopeDestroy       : function (Prm : Integer) : Integer; cdecl;
     ScopeShow          : function (Prm : Integer) : Integer; cdecl;
     ScopeHide          : function (Prm : Integer) : Integer; cdecl;
     ScopeShowNext      : function (Prm : Integer; PrmD : PTArrDbl) : Integer; cdecl;
     ScopeQuickUpDate   : function (Prm : Integer) : Integer; cdecl;
*/

/////////////////////////////////////////////////////////////////////////////
// CVC_TSTDlg dialog

CVC_TSTDlg::CVC_TSTDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CVC_TSTDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CVC_TSTDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	fLoaded = 0;
}

void CVC_TSTDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CVC_TSTDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CVC_TSTDlg, CDialog)
	//{{AFX_MSG_MAP(CVC_TSTDlg)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_LOAD, OnButtonLoad)
	ON_BN_CLICKED(IDC_BUTTON_RUN, OnButtonRun)
	ON_WM_CLOSE()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CVC_TSTDlg message handlers

BOOL CVC_TSTDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	// TODO: Add extra initialization here
	fThreadFinished = 1;
DllInst=0;

	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CVC_TSTDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CVC_TSTDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CVC_TSTDlg::OnButtonLoad() 
{
	// TODO: Add your control notification handler code here
  if (fLoaded==0)
  {
	ScopeAfterOpenLib = NULL;
    DllInst = ::LoadLibrary((char *)DllName);
	if (DllInst==NULL)
	{
		MessageBox("Error DLL loading 1","Error",MB_ICONSTOP);
     exit(-1);  
    } 



   ScopeAfterOpenLib  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "AtOpenLib");
   if (ScopeAfterOpenLib == NULL) 
   { FreeLibrary(DllInst); MessageBox("Error DLL loading 2","Error",MB_ICONSTOP); 
     exit(-1);  
   }
   ScopeAfterOpenLib (0);



   ScopeShowNext  = (int (__cdecl  * )(int, struct tagTArrDbl *))GetProcAddress(DllInst, "ShowNext");
   if (ScopeShowNext == NULL) 
   { FreeLibrary(DllInst); MessageBox("Error DLL loading 3","Error",MB_ICONSTOP); 
     exit(-1);  
   }

   ScopeCreate  = (int (__cdecl  * )(int, char *, char *))GetProcAddress(DllInst, "ScopeCreate");
   if (ScopeCreate == NULL) 
   { FreeLibrary(DllInst); MessageBox("Error DLL loading 4","Error",MB_ICONSTOP); 
     exit(-1);  
   }

   
   ScopeDestroy  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeDestroy");
   if (ScopeDestroy == NULL) 
   { FreeLibrary(DllInst); MessageBox("Error DLL loading 5","Error",MB_ICONSTOP); 
     exit(-1);  
   }

   ScopeShow  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeShow");
   if (ScopeShow == NULL) 
   { FreeLibrary(DllInst); MessageBox("Error DLL loading 6","Error",MB_ICONSTOP); 
     exit(-1);  
   }

   ScopeHide  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeHide");
   if (ScopeHide == NULL) 
   { FreeLibrary(DllInst); MessageBox("Error DLL loading 7","Error",MB_ICONSTOP); 
     exit(-1);  
   }


   ScopeQuickUpDate  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "QuickUpDate");
   if (ScopeQuickUpDate == NULL) 
   { FreeLibrary(DllInst); MessageBox("Error DLL loading 8","Error",MB_ICONSTOP); 
     exit(-1);  
   }

   

    Scopes[1] = ScopeCreate(0, pEmpty, pEmpty);
    ScopeShow(Scopes[1]);


	fLoaded = 1;fRun=0;
  }
  else
  {

 // ScopeDestroy(Scopes[0]);
  ScopeDestroy(Scopes[1]);

  FreeLibrary(DllInst);
  
  ScopeAfterOpenLib = NULL;
  ScopeShowNext     = NULL;
  ScopeCreate       = NULL;
  ScopeQuickUpDate  = NULL;
  ScopeHide         = NULL;
  ScopeShow         = NULL;
  ScopeDestroy      = NULL;

	fLoaded = 0;fRun=0;
DllInst=0;
  }
 /*

 end
 else
 begin
 
  ScopeDestroy(Scopes[1]);
  ScopeDestroy(Scopes[2]);

  FreeLibrary(DllInst);
  
  ScopeAfterOpenLib := NIL;
  ScopeShowNext     := Nil;
  ScopeCreate       := Nil;
  ScopeQuickUpDate  := Nil;
  ScopeHide         := Nil;
  ScopeShow         := Nil;
  ScopeDestroy      := Nil;
  BtnLoad.Caption := 'Load Scope';
 end;
*/


}


HANDLE TstTrd;

DWORD WINAPI OSCThreadFunc( LPVOID )
{
struct tagTArrDbl D,D1;

     D.s1 = 0 ; D.s2 = 0; D.s3 = 0;
     D1.s1 = 0 ; D1.s2 = 0; D1.s3 = 0;

if (fLoaded==0) ExitThread(11);


while (fRun==1)
{

    D1.s1 = D1.s1 - 11;         if (D1.s1 < -280)  D1.s1 = 20;
    D1.s2 = D1.s2 - 3.1333;     if (D1.s2 < -280)  D1.s2 = 20;
    D1.s3 = D1.s3 - 5.1333;     if (D1.s3 < -280)  D1.s3 = 20;
    ScopeShowNext( Scopes[1], &D1 );
}

 ScopeQuickUpDate(Scopes[1]);


 fThreadFinished = 1;


 ExitThread(	11);

 return 0;
};


void CVC_TSTDlg::OnButtonRun() 
{
	// TODO: Add your control notification handler code here
	
	if (fRun==0) 
	{
   fRun = 1;
   fThreadFinished = 0;

TstTrd = CreateThread(

    0,	// pointer to thread security attributes  
    0,	// initial thread stack size, in bytes 
    OSCThreadFunc,	// pointer to thread function 
    NULL,	// argument for new thread 
    0,	// creation flags 
    NULL	// pointer to returned thread identifier 
   );

  // TstTrd.Priority := tpHigher;

int hhh = SetThreadPriority(
TstTrd,	// handle to the thread 
    THREAD_PRIORITY_NORMAL 	// thread priority level 
   );

  //TstTrd.Resume;
	}
	 else
	 {
		    fRun=0;

//				Sleep(100);
//           while (fThreadFinished != 1)
//{
//				Sleep(100);
//			}
//			CloseHandle(TstTrd);
//			TstTrd  =  0 ;
	 }
  


}

void CVC_TSTDlg::OnClose() 
{
	// TODO: Add your message handler code here and/or call default
	fRun=0;
	CDialog::OnClose();
}
