function f = obj_fun_logvol(v, P, n, k)
 % v: 2n vector [x1;y1; x2;y2; ...] flattened as column-major from reshape
    xy = reshape(v, 2, n);
    N = normals_from_xy(xy);
    if k == 1
        B = AllocationMatrixNoDrag(N, P);
    else
        B = AllocationMatrixComplete(N, P);
    end    
    s = svd(B);
    f = -sum(log(s(1:6) + 1e-12));
end