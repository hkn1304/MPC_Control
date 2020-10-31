function [A, b] = build_state_constraint_matrix(Ac, Bc, N, x_min, x_max, x)
x_max_vect = repmat(x_max, N, 1)
b = x_max_vect - Ac * x
A = [Bc]

x_min_vect = repmat(x_min, N, 1);
A = [A; -[Bc]];
b = [b; - x_min_vect + Ac * x];
end