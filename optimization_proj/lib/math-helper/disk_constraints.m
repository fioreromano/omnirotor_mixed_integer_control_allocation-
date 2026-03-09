function [c, ceq] = disk_constraints(v, n, eps_margin)
    % inequality constraints c(v) <= 0
    % we want: x_j^2 + y_j^2 - (1 - eps_margin) <= 0  -> so c = x^2+y^2 - (1-eps)
    xy = reshape(v, 2, n);
    x = xy(1,:);
    y = xy(2,:);
    c_disk = (x.^2 + y.^2) - (1 - eps_margin);
    c = c_disk;
    ceq = [];                                % no equality constraints
end