function W_map = AllocationMatrixComplete(N, P)
    % N: 3 x 8
    % P: 3 x 8
    coef = [-1 1 -1 1 -1 1 -1 1]; 
    k = 0.0147;
    crossPN = cross(P, N, 1); 
    spin_term = N .* coef;              
    moment_block = crossPN + k*spin_term;
    W_map = [N; moment_block];
end