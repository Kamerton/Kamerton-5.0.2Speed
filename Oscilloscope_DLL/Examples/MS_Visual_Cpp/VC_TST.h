// VC_TST.h : main header file for the VC_TST application
//

#if !defined(AFX_VC_TST_H__5AEA9B55_8EDC_4C25_B13C_D506919F9FAB__INCLUDED_)
#define AFX_VC_TST_H__5AEA9B55_8EDC_4C25_B13C_D506919F9FAB__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CVC_TSTApp:
// See VC_TST.cpp for the implementation of this class
//

class CVC_TSTApp : public CWinApp
{
public:
	CVC_TSTApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CVC_TSTApp)
	public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CVC_TSTApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_VC_TST_H__5AEA9B55_8EDC_4C25_B13C_D506919F9FAB__INCLUDED_)
