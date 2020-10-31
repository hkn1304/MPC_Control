%% MPC HW_Question 6: 2 States, 1 Input
sys.a=[.9 1; 0 .5]; % A: [#ofStates , #ofStates]
sys.b=[1;0];        % B: [#ofStates , #ofInputs]
N=3; % N: Prediction Horizon
Nu=2; % Nu: Control Horizon
Q=1; R=.1; % Weighting coeff.
x=[1;1] % Initial state

Ac = [];
for i = 1:N
    Ac = [Ac; sys.a^i] % Ac: [#ofStates*N , #ofStates]
end

Bc = [];
for i= 1:N
    line = [];
    for j = 1:N
        if(j>i)
            el = zeros(size(sys.b));
        else
            el = sys.a^(i-j) * sys.b;
        end
        line = [line el];
    end
    Bc = [Bc;line]; % [ #ofStates*N, #ofInputs*N]
end

Qc=eye(6)
Rc = kron(eye(N), R) % Rc: [N , N]
H = 2 * (Bc' * Qc * Bc + Rc)  % [N , N]
f = 2 * x' * Ac' * Qc * Bc  % %[#ofStates , N]

[A_state, b_state] = build_state_constraint_matrix(Ac, Bc, N, [-10;-10], [10;10], [1;1])
[A_input, b_input] = build_input_constraint_matrix(Nu, -1, 1)
A=[A_state;A_input];
b=[b_state;b_input];
[U,fval,exitflag]=quadprog(H,f,A,b)




