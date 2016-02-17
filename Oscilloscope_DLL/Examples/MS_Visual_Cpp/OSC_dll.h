// File : OSC_DLL.H

 
#define DllName  "Osc_DLL.dll"

//extern  char *pEmpty = "";


struct tagTArrDbl {
double s1;
double s2;
double s3;
} ;

extern int (__cdecl  * ScopeAfterOpenLib)(int Prm);
extern int (__cdecl  *     ScopeCreate)   (int Prm ,char  *P_IniName, char *P_IniSuffix);
extern int (__cdecl  *     ScopeDestroy)  (int Prm);
extern int (__cdecl  *     ScopeShow)      (int Prm);
extern int (__cdecl  *     ScopeHide)       (int Prm);
extern int (__cdecl  *     ScopeShowNext)    (int Prm, struct  tagTArrDbl *PrmD );
extern int (__cdecl  *     ScopeQuickUpDate)  (int Prm);
