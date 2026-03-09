function f = obj_fun(v, P, n, k)
 % v: 2n vector [x1;y1; x2;y2; ...] flattened as column-major from reshape
    xy = reshape(v, 2, n);
    N = normals_from_xy(xy);
    if k == 1
        B = AllocationMatrixNoDrag(N, P);
    else
        B = AllocationMatrixComplete(N, P);
    end    
    s = svd(B);
    sigma_min = min(s);
    f = -sigma_min + 10*(max(s) / min(s));
end