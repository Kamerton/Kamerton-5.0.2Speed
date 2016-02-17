//---------------------------------------------------------------------------

#ifndef Unit2H
#define Unit2H
//---------------------------------------------------------------------------
#include <Classes.hpp>
//---------------------------------------------------------------------------
class TTstTrd : public TThread
{            
private:
protected:
        void __fastcall Execute();
public:
        __fastcall TTstTrd(bool CreateSuspended);
};
//---------------------------------------------------------------------------
#endif
 