%% Model definition
% x1: angle of attack
% x2: pitch angle
% x3: pitch rate
% x4: altitude
% y1: putch angle
% y2: altitude

A = [-1.2822, 0, 0.98, 0; 0, 0, 1, 0; -5.4293, 0, -1.8366, 0; -128.2, 128.2, 0, 0];
B = [-0.3; 0; -17; 0];
C = [0, 1, 0, 0; 0, 0, 0, 1];
D = [0; 0];
Ts = 0.25;
sys = ss(A, B, C, D);
OL_poles= pole(sys)
pzmap(sys)
stability=isstable(sys);
if(stability)
else
display('The open loop system is unstable!');
end

sys_discrete = c2d(sys, Ts);

%% Parameters
u_min = -0.262;
u_max= 0.262;
u_rate_min = -0.524;
u_rate_max = 0.524;
x2_min = -0.349;
x2_max = 0.349;

%% Open Loop Analysis
if(rank(ctrb(sys_discrete)) == 4 && rank(obsv(sys_discrete.a,sys_discrete.c)) == 4)
    display('Rank of controllability and observability matrix is 4 -> the system is controllable and observable');
end
pzmap(sys_discrete)

%% a) LQR controller
display('LQR controller');
close all;
Q = eye(4)
R = 10
[Kdlqr, S, closeLoopEigs] = dlqr(sys_discrete.a, sys_discrete.b, Q, R);
theorem = (rank(ctrb(sys_discrete)) == 4) && (all(eig(S)>0));
if(theorem)
    display('The assumptions for the asymptotical stability of LQR are verified');
end
display(['Closed loop matrix A-BK eignevalues = ' num2str(closeLoopEigs')]);
if(abs(closeLoopEigs) < 1)
    display('Eigenvalues inside unit circle -> closed loop system is AS.')
else
    display('Eigenvalues outside unit circle -> error.')
end

%% LQR No input saturation
T_sim=20; 
set_point=[10;10;0;0]
x0=[0;0;0;10]
display('LQ controller simulation');
u_min_simulink = -inf
u_max_simulink = inf
Kdlqr_simulink = Kdlqr
sim('LQR_discrete_hkn');
open('LQR_discrete_hkn');

%% Adding input saturation to LQR regulator
display('Adding input saturation to LQR regulator');
set_point=[5000;10;1000;0]
u_min_simulink = u_min
u_max_simulink = u_max
Kdlqr_simulink = Kdlqr
sim('LQR_discrete_hkn');
open('LQR_discrete_hkn');

%% b)MPC Controller design 
mpcHW7 = mpc(sys_discrete, Ts);

mpcHW7.PredictionHorizon = 10; % specify prediction horizon
mpcHW7.ControlHorizon = 3; % specify control horizon

% specify nominal(initial) values for inputs and outputs
mpcHW7.Model.Nominal.U = 0;
%mpcHW7.Model.Nominal.Y = [0;10];
mpcHW7.Model.Nominal.X = set_point;
% specify constraints for MV and MV Rate
mpcHW7.MV(1).Min = -Inf; %u_min
mpcHW7.MV(1).Max = Inf;  %u_max
mpcHW7.MV(1).RateMin = -Inf; %u_rate_min
mpcHW7.MV(1).RateMax = Inf;  %u_rate_max

% specify weights
mpcHW7.Weights.MV = 10; % R=10
mpcHW7.Weights.MVRate = 1;
mpcHW7.Weights.OV = [1 1]; % Q=eye

% specify simulation options
options = mpcsimopt();
options.PlantInitialState = set_point;
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

% run simulation
refs=[100 0];
simtime= 81*Ts; % 41*Ts=10 sn
sim(mpcHW7,81,refs,options);

% specify constraints for MV (Input constraints)
mpcHW7.MV(1).Min = u_min;
mpcHW7.MV(1).Max = u_max;
mpcHW7.MV(1).RateMin = -Inf;
mpcHW7.MV(1).RateMax = Inf;
sim(mpcHW7,81,refs,options);

%% c) specify constraints for MV (Input constraints) and MV Rate (Input rate constraints)
mpcHW7.MV(1).Min = u_min;
mpcHW7.MV(1).Max = u_max;
mpcHW7.MV(1).RateMin = u_rate_min;
mpcHW7.MV(1).RateMax = u_rate_max;
sim(mpcHW7,41,refs,options);

% specify nominal(initial) values for inputs and outputs
mpcHW7.Model.Nominal.U = 0;
mpcHW7.Model.Nominal.Y = [0;100]; %x0 at 100 m altitude
sim(mpcHW7,41,refs,options);

%% d) Add state constraints
[A_state, b_state] = build_state_constraint_matrix(sys_discrete, mpcHW7.PredictionHorizon, x2_min, x2_max, repmat(mpcHW7.Model.Nominal.U,4,1))
E=[diag(A_state);diag(A_state,-10)]
G=[b_state]
setconstraint(mpcHW7,E,[],G,[],[])

sim(mpcHW7,41,refs,options);

%% e) Decrease prediction horizon to 4 and show that this decrease may cause loss of stability
mpcHW7.PredictionHorizon = 4;
sim(mpcHW7,41,refs,options);

%% f)Terminal constraints
Y = struct('Weight',[1,10],'Min',[0,-Inf],'Max',[Inf,2]);
U = struct('Min',10,'Max',100);

setterminal(mpcHW7,Y,U)
sim(mpcHW7,41,refs,options);




