function W = WMat(r)
% Compute the left Jacobian of SO(3).

    mag = norm(r);
    r_skew = skewMat(r);
    if mag == 0
        W = eye(3);
    else
        W = eye(3) - ((1-cos(mag))/(mag^2)) * r_skew + ((mag-sin(mag))/(mag^3)) * r_skew^2;
    end

end