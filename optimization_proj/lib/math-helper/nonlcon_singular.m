function [c, ceq] = nonlcon_singular(x, cf, ct, P, lambda, w_des, u_max)

    % Compute wrench map
    alpha = x(1:8);
    beta = x(9:16);
    W_map = myWrenchMapVariableTilt(x, cf, ct, P, lambda);

    % ------------------------------------------
    % (1) SINGULAR VALUE CONSTRAINT
    % ------------------------------------------
    sigma_min = min(svd(W_map));
    epsilon = 1e-6;
    c1 = epsilon - sigma_min;

    u_grav = pinv(W_map) * w_des';
    % rotor thrusts must be non-negative
    c2 = -min(u_grav);      % ensures min(u_grav) >= 0  -> c2 <= 0
    % rotor thrusts must be <= u_max
    c3 = max(u_grav) - u_max;  % ensures max(u_grav) <= u_max
    % Combine all inequality constraints
    c = [c1; c2; c3];

    % ceq = [
    %         alpha(1) - alpha(3); 
    %         alpha(2) - alpha(4);
    %         alpha(5) - alpha(7); 
    %         alpha(6) - alpha(8);
    %         beta(1) - beta(3);
    %         beta(2) - beta(4);
    %         beta(5) - beta(7);
    %         beta(6) - beta(8)
    % ];

    ceq = [];
end
