function W_map = AllocationMatrixNoDrag(N, P)
    % N: 3 x 8
    % P: 3 x 8
    crossPN = cross(P, N, 1); 
    moment_block = crossPN;     % Brescianini semplified form
    W_map = [N; moment_block];
end