function W_map = myWrenchMapVariableTilt(x, cf, ct, P, lambda)
    alpha = x(1:8);
    beta  = x(9:16);
    coef = [-1 1 -1 1 -1 1 -1 1];    
    W_map = zeros(6,8);
    z0 = [0;0;1];

    for i = 1:8        
        % Complete rotation: R_Z(λ_i) R_X(α_i) R_Y(β_i)
        
        Rx = [1, 0, 0;
              0, cos(alpha(i)), -sin(alpha(i));
              0, sin(alpha(i)), cos(alpha(i))];
        
        Ry = [cos(beta(i)), 0, sin(beta(i));
              0, 1, 0;
              -sin(beta(i)), 0, cos(beta(i))];
        
        Rz = [cos(lambda(i)), -sin(lambda(i)), 0;
              sin(lambda(i)), cos(lambda(i)), 0;
              0, 0, 1];

        R = Rz * Rx * Ry;

        z_i = R * z0;
        f_i = cf * z_i;       
        m_i = cf * cross(P(i,:)', z_i) - coef(i) * ct * z_i;

        W_map(:,i) = [f_i; m_i];
    end
end