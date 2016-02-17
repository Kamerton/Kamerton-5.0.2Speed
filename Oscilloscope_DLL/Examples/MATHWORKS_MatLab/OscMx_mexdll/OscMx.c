// How to build in Matlab command window:
//   mex OscMx.c


#include <windows.h>
#include <mex.h>
// OscMx.dll
// -------------------------------------------
//#include "OSC_DLL.H"
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
/////////////////

extern char * pEmpty = "";



char const EmptyM[4] = {0,0,0,0};

    // DLL functions begin
int (__cdecl  * ScopeAfterOpenLib)(int Prm);
int (__cdecl  *     ScopeCreate)   (int Prm ,char  *P_IniName, char *P_IniSuffix);
int (__cdecl  *     ScopeDestroy)  (int Prm);
int (__cdecl  *     ScopeShow)      (int Prm);
int (__cdecl  *     ScopeHide)       (int Prm);
int (__cdecl  *     ScopeShowNext)    (int Prm, struct  tagTArrDbl *PrmD );
int (__cdecl  *     ScopeQuickUpDate)  (int Prm);

HINSTANCE DllInst;
int Scopes[7];

/*
void OnOscLoad() 
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



}
*/

BOOL LoadOscDllFunctions()
{

   ScopeAfterOpenLib  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "AtOpenLib");
   if (ScopeAfterOpenLib == NULL) 
   { mexErrMsgTxt( "Error -2" ) ; 
     return  FALSE;
   }
   ScopeAfterOpenLib (0);



   ScopeShowNext  = (int (__cdecl  * )(int, struct tagTArrDbl *))GetProcAddress(DllInst, "ShowNext");
   if (ScopeShowNext == NULL) 
   {  mexErrMsgTxt( "Error -2" ) ; 
     return  FALSE;
   }

   ScopeCreate  = (int (__cdecl  * )(int, char *, char *))GetProcAddress(DllInst, "ScopeCreate");
   if (ScopeCreate == NULL) 
   { mexErrMsgTxt( "Error -2" ) ; 
     return  FALSE;
   }

   
   ScopeDestroy  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeDestroy");
   if (ScopeDestroy == NULL) 
   { mexErrMsgTxt( "Error -2" ) ; 
     return  FALSE;
   }

   ScopeShow  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeShow");
   if (ScopeShow == NULL) 
   {  mexErrMsgTxt( "Error -2" ) ; 
     return  FALSE;
   }

   ScopeHide  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeHide");
   if (ScopeHide == NULL) 
   {  mexErrMsgTxt( "Error -2" ) ; 
     return  FALSE;
   }


   ScopeQuickUpDate  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "QuickUpDate");
   if (ScopeQuickUpDate == NULL) 
   {  mexErrMsgTxt( "Error -2" ) ; 
     return  FALSE;

   }
return TRUE;
}


// Here is the mex interface 
//extern "C"
void mexFunction(int nlhs, mxArray * plhs[], int nrhs, mxArray * prhs[])
/* Parameters:
Par1:

	   -1          Unload library
		0          Load library
        1          AtOpenLib,
        2          ScopeCreate,
        3          ScopeDestroy,
        4          ScopeShow,
        5          ScopeHide,
        6          QuickUpDate,
        7          ShowNext;

*/


{
long lPar1;
double par1;
double *out ;
double retval;
double *dptr;
unsigned long cnt;

//AFX_MANAGE_STATE(AfxGetStaticModuleState());
				struct tagTArrDbl D;

	if ( (nrhs >= 1) && prhs[0]  )
	{
		par1 = *(double*)(mxGetPr(prhs[0]));
		lPar1 = (long)par1;
		switch (lPar1)
		{
		case 2: // Scope create
//			CheckVir();

			if (Scopes[1]==NULL)
			{
			Scopes[1] = ScopeCreate(0, "Scope_Desk.ini", "");
			}
			retval=Scopes[1];
			break;
		case 3: // Scope destroy
			if (Scopes[1])
			{
			ScopeDestroy(Scopes[1]);
			Scopes[1]=0;
			retval=0;
			}
			break;
		case 4: // ScopeShow
			if (Scopes[1])
			{
			ScopeShow(Scopes[1]);
			retval=0;
			}
			break;
		case 5: // ScopeHide
			if (Scopes[1])
			{
			ScopeHide(Scopes[1]);
			retval=0;
			}
			break;
		case 6: // ScopeQuickUpDate
			if (Scopes[1])
			{
			ScopeQuickUpDate(Scopes[1]);
			retval=0;
			}
			break;
		case 7: // ScopeShowNext
			retval = 333;
			if ( (nrhs >= 2) && prhs[1]  )
			{
				dptr = mxGetPr(prhs[1]);
				D.s1 = dptr[0];
				D.s2 = dptr[1];
				D.s3 = dptr[2];

				if (Scopes[1])
				{
				retval=ScopeShowNext( Scopes[1], &D);
				//mlfDrawnow();
				//WaitMessage();
				//GetMessage(&msg,NULL,NULL,NULL);
				//TranslateMessage(&msg);
				//DispatchMessage(&msg);
				}
		
			}
			break;
		case 8: // ScopeShowNextVec
			retval = 333;
			if ( (nrhs >= 2) && prhs[1]  )
			{
				unsigned long len = mxGetM(prhs[1]);
				dptr = mxGetPr(prhs[1]);
				
				for (cnt=0; cnt < len; cnt++)
				{
				D.s1 = dptr[cnt];
				D.s2 = dptr[cnt+len];
				D.s3 = dptr[cnt+2*len];

				 if (Scopes[1])
				 {
				 retval=ScopeShowNext( Scopes[1], &D);
				 //mlfDrawnow();
				 //WaitMessage();
				 //GetMessage(&msg,NULL,NULL,NULL);
				 //TranslateMessage(&msg);
				 //DispatchMessage(&msg);
				 }
				}
		
			}
			//mlfDrawnow(NULL);

			break;
		


			
		default: ;
		}
	}

	plhs[0] = mxCreateDoubleMatrix( 1 , 1 , mxREAL ) ; 
	out = mxGetPr( plhs[0] ) ;
	*out = (unsigned long)retval ;
	
}




BOOL InitInstance() 
{
	// TODO: Add your specialized code here and/or call the base class
		ScopeAfterOpenLib = NULL;
		ScopeShowNext     = NULL;
		ScopeCreate       = NULL;
		ScopeQuickUpDate  = NULL;
		ScopeHide         = NULL;
		ScopeShow         = NULL;
		ScopeDestroy      = NULL;
	Scopes[1] = 0;

	
	DllInst =  LoadLibrary((char *)DllName);
	if (DllInst==NULL)
	{
					mexErrMsgTxt( "Error -1" ) ;
					return FALSE;
	} 
	if (LoadOscDllFunctions()==FALSE)
	{
		DllInst=NULL;
		return FALSE;
	}


	return 0;
}

int ExitInstance() 
{
	// TODO: Add your specialized code here and/or call the base class
	if (DllInst)
	{
			if (Scopes[1])
			{
			ScopeDestroy(Scopes[1]);
			Sleep(500);
			}

		FreeLibrary(DllInst);
	}

	return 0;
}

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{

    switch (ul_reason_for_call)
	{
	 case DLL_PROCESS_ATTACH:
         InitInstance();
			break;
		case DLL_THREAD_ATTACH:
			break;
		case DLL_THREAD_DETACH:
			break;
		case DLL_PROCESS_DETACH:
            ExitInstance();
			break;
    }
    return TRUE;
}

