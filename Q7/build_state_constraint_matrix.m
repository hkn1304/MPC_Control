function [A, b] = build_state_constraint_matrix(sys, N, x_min, x_max, x0)
Ac = [];
for i = 1:N
    Ac = [Ac; sys.a^i]
end

Bc = [];
for i= 1:N
    line = [];
    for j = 1:N
        if(j>i)
            el = zeros(size(sys.b))
        else
            el = sys.a^(i-j) * sys.b
        end
        line = [line el]
    end
    Bc = [Bc;line]
end

Ac= Ac(2:4:end,:)
Bc= Bc(2:4:end,:)

x_max_vect = repmat(x_max, N, 1)
b = x_max_vect - Ac * x0
A = [Bc]

x_min_vect = repmat(x_min, N, 1);
A = [A; -[Bc]];
b = [b; - x_min_vect + Ac * x0];
end


