// VC_TSTDlg.h : header file
//

#if !defined(AFX_VC_TSTDLG_H__4C3C8D0D_3484_4595_9A8F_58BF63A4C5FB__INCLUDED_)
#define AFX_VC_TSTDLG_H__4C3C8D0D_3484_4595_9A8F_58BF63A4C5FB__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CVC_TSTDlg dialog

class CVC_TSTDlg : public CDialog
{
// Construction
public:
	CVC_TSTDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CVC_TSTDlg)
	enum { IDD = IDD_VC_TST_DIALOG };
		// NOTE: the ClassWizard will add data members here
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CVC_TSTDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CVC_TSTDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnButtonLoad();
	afx_msg void OnButtonRun();
	afx_msg void OnClose();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_VC_TSTDLG_H__4C3C8D0D_3484_4595_9A8F_58BF63A4C5FB__INCLUDED_)
