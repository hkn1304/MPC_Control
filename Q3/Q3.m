%%%%%% Define plant and internal model:
A = [0.9 .5 ; 0 1]; B = [0.5 ; 0]; % Define the plant with
C = [1 0]; D = 0; % disturbance as in part (a)
pmod = ss2mod(A,B,C,D); % Put plant model into MOD format
imod = pmod; % Internal model same as plant model
%%%%%% Compute predictive controller:
Q = 1; % penalty on plant output errors
R = 0; % penalty on input moves
Hp = 30; Hu = 2; % Prediction and control horizons respectively
Ks = smpccon(imod,Q,R,Hu,Hp) % Compute controller gain matrix

%%%%%% Simulate with deadbeat observer:
Lprime = [1 ; 2 ; 1]; % Observer gain L' as calculated above
tend = 30; % End time for simulation
r = 1; % Set-point = 1 (constant)
wu=[zeros(1,10),1]'; % Unmeasured step disturbance on the
% input after 10 time steps
[y1,u1] = smpcsim(pmod,imod,Ks,tend,r,[],Lprime,[],[],[],wu);
plotall(y1,u1)
title('Deadbeat observer MV')

%%%%%% Simulate with default observer:
[y2,u2] = smpcsim(pmod,imod,Ks,tend,r,[],[],[],[],[],wu);
figure; plotall(y2,u2)
title('DMC observer MV')