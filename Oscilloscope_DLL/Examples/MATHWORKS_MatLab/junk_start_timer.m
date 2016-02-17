% junk_start_timer
% first try find timers
secPeriod = 0.001;

OldTimers = timerfind('Tag', 'BeTimer');
if ( ~isempty(OldTimers))
    try
        stop(OldTimers);
    catch
        str =lasterr;
        disp(str);
    end
    
    delete(OldTimers);
end;

t = timer('Name','OwPxTimer','Tag','BeTimer','TimerFcn',@cbMyTimer, 'Period', secPeriod,'ExecutionMode','fixedSpacing');
start(t);
