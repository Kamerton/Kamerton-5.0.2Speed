// VC_TST.cpp : Defines the class behaviors for the application.
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

/////////////////////////////////////////////////////////////////////////////
// CVC_TSTApp

BEGIN_MESSAGE_MAP(CVC_TSTApp, CWinApp)
	//{{AFX_MSG_MAP(CVC_TSTApp)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG
	ON_COMMAND(ID_HELP, CWinApp::OnHelp)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CVC_TSTApp construction

CVC_TSTApp::CVC_TSTApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}

/////////////////////////////////////////////////////////////////////////////
// The one and only CVC_TSTApp object

CVC_TSTApp theApp;

/////////////////////////////////////////////////////////////////////////////
// CVC_TSTApp initialization
extern HINSTANCE DllInst;
extern int fRun;
extern HANDLE TstTrd;
extern int fThreadFinished;
extern int Scopes[7];

BOOL CVC_TSTApp::InitInstance()
{
	// Standard initialization
	// If you are not using these features and wish to reduce the size
	//  of your final executable, you should remove from the following
	//  the specific initialization routines you do not need.

	Scopes[1] = 0;
	CVC_TSTDlg dlg;
	m_pMainWnd = &dlg;
	int nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: Place code here to handle when the dialog is
		//  dismissed with OK
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: Place code here to handle when the dialog is
		//  dismissed with Cancel
	}

	// Since the dialog has been closed, return FALSE so that we exit the
	//  application, rather than start the application's message pump.

 	 fRun=0;
	 
     //::SuspendThread(TstTrd);

	 while (fThreadFinished != 1)
	 {
	 ;
	 }
			//CloseHandle(TstTrd);
			//TstTrd  =  0 ;

	if (DllInst)
    {
    if (Scopes[1] != 0) ScopeDestroy(Scopes[1]);

	FreeLibrary(DllInst);
    }







	return FALSE;
}

int CVC_TSTApp::ExitInstance() 
{
	// TODO: Add your specialized code here and/or call the base class

return 	CWinApp::ExitInstance();

}
