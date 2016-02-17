% RunMic2_v5
%Purpose: run simulation
%IMPORTANT:
% 1.
% The first parameter for each block OscS in a model must be unique
% index in range [0,1,2]
% 2. The 2nd parameter ModelScopeIniFileName must be global for entire
% model.
% 3. If you use OscMx.dll (openscope etc. functions), first use command
% clear OscS to close scope windows

ModelScopeIniFileName='mic_c2_scope.ini'; % Name of the ini file used in this model
ModelScopeReload=1; % 0-do not reload library if run model with different  ModelScopeIniFileName
ModelScopeClearBuffer=1; % 1-clear scope buffer each run
sim('mic_controller');

pause


ModelScopeIniFileName='mic_c_scope.ini'; % Name of the ini file used in this model
ModelScopeReload=1; % 0-do not reload library if run model with different  ModelScopeIniFileName
ModelScopeClearBuffer=1; % 1-clear scope buffer each run
sim('mic_rl');




