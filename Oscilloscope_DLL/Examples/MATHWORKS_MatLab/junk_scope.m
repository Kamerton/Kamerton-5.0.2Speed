% oscmx(0);
% oscmx(2);
% oscmx(4);

% sys1 = tf(1,[1 100]);
% mag=bode(sys1,logspace(-1,7));
% for cnt=1:length(mag)
%     oscmx(7,[mag(cnt),-mag(cnt),2*mag(cnt)]);
% end
% 

figure (1); plot ([1:1024],sin([1:1024]/512));
cnt=1;
tic
while (cnt<100000)
    cnt = cnt+1;
    oscmx(8,[100* sin(cnt*2*pi / 1000+rand(1) ),100*cos(cnt*2*pi / 1000 ),-100*sin(2*cnt*2*pi / 1000 )  ]);
    % pause(0.00001);
    drawnow;
    
end
toc
tic
t = [1:100000];
r = 100* sin(t*2*pi / 1000+rand(1) );
g = 100*cos(t*2*pi / 1000 );
b = -100*sin(2*t*2*pi / 1000 );
for cnt=1:length(t)
    r(cnt)=100* sin(t(cnt)*2*pi / 1000+rand(1) );
end

for kk=1:1 
oscmx(8,  [r' , g' , b']);
end

toc

