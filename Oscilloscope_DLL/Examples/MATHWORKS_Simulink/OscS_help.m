% OscS S-function
% Simulink interface for Osc_dll.dll
% Compartible with matlab R11,R13,R14
%Must be used with 4 dialog box parameters:
% 1. Scope window index: unique index in range  [0,1,2]  
% 2. Ini-file name: string, global for entire model
% 3. Flag, if you want to reload library when another model run, that uses
% different Ini-file. 
%    flag = 1 (recommended): the library will be reloaded using new ini-file.  
%    flag = 0 : the library is not reloaded and current ini-file is used.
%    The last option decreases the start time time.
% 4. Flag, if you want to clear the scope buffer every model run.
%    1: clear the scope
%    0: do not clear the scope buffer. Old simulation results are stored in
%    the scope buffer.
%
% It is suggested, that model uses a single scope ini file for the
% definition  of all scope windows. The first parameter is used to select
% the window (Oscilloscope_0,Oscilloscope_1, etc). If you want to save the scope setting, you must
% save it to this single file (using the scope Menu button) manually for each window. 
% (Remember to pess 'connect' button after the file is saved !)
%
%    Known problems
%  If you want to use OscMx.dll (openscope etc. functions), first apply the command
%                 clear OscS
%  to close scope windows.
%  If you used OscMx.dll and now want to use simulink with OscS, first
%  apply command 
%                 clear OscM
% It is recommended to use only OscS, see RunScopeOnly example.




