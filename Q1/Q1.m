%% MPC Midterm Homework Q1

num=[0 0 0 0.2713]
denum=[1 -0.8351 0 0]
%Obtain the impulse response
impresponse= impz(num,denum,30)

% The Step Response(sr) from the impulse response
p.sr=zeros(30,1);
for i=1:30
    for j=1:i
    p.sr(i)=p.sr(i)+impresponse(j)
    end
end


% Setup the DMC
N=120;  % Total simulation length (samples)
p.p=10; % Prediction horizon
p.m=5;  % Moving horizon
p.la=1; % Control weight
% Reference (setpoint)
R=[ones(30,1);zeros(30,1);ones(30,1);zeros(30,1)];
p.y=0;  % Initial output
p.v=[]; % empty past input to indicate initialization
% buffer of input to cope with time delay
u=zeros(3,1);
% Initialization of variables for results
Y=zeros(N,1);
U=zeros(N,1);
% DMC Simulation
for k=1:120
    p.a=0;
    p.r=R(k);   % DMC only knows current setpoint
    if k>60     % change smoothing factor for second half simulation
        p.a=0.7;
    end
    p=dmc(p);
    Y(k)=p.y;
    U(k)=p.u;
    u=[u(2:3);p.u];
    p.y=0.8351*p.y+0.2713*u(1); % actual plant output 
end
% DMC results
subplot(211)
plot(1:N,Y,'b-',1:N,R,'r--',[60 60],[-0.5 1.5],':','linewidth',2)
title('solid: output, dashed: reference')
text(35,1,'\alpha=0')
text(95,1,'\alpha=0.7')
axis([0 120 -0.5 1.5])
subplot(212)
[xx,yy]=stairs(1:N,U);
plot(xx,yy,'-',[60 60],[-0.5 1.5],':','linewidth',2)
axis([0 120 -0.5 1.5])
title('input, \lambda=1')
xlabel('time, min')
