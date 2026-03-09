function N = normals_from_xy(xy)
    % xy: 2 x n
    x = xy(1,:);
    y = xy(2,:);
    r2 = x.^2 + y.^2;
    % numerical safety: clip to [0,1]
    r2 = min(r2, 1.0);
    z = sqrt(max(0, 1 - r2));
    N = [x; y; z];
end