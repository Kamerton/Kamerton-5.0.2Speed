function cbMyTimer(obj,event)
global cnt

if isempty(cnt)
    cnt=0;
end;

    cnt = cnt+1;
    oscmx(7,[100* sin(cnt*2*pi / 1000+rand(1) ),100*cos(cnt*2*pi / 1000 ),-100*sin(2*cnt*2*pi / 1000 )  ]);
