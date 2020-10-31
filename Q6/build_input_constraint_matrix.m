function [A, b] = build_input_constraint_matrix(N, u_min, u_max)
A=[eye(N) zeros(N,1);-eye(N) zeros(N,1)]
u_max_vect = repmat(u_max, N, 1)
b = u_max_vect 

u_min_vect = repmat(u_min, N, 1)
b = [b; - u_min_vect];
end
