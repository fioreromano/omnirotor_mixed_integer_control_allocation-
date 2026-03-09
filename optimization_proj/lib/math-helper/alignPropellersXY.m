function [lambda_opt] = alignPropellersXY(p_B, r_CoM_B)

    N = size(p_B,1);
    lambda_opt = zeros(N,1);

    for i = 1:N
        % vettore motore -> CoM
        d = r_CoM_B - p_B(i,:)';
        d_xy = d(1:2) / norm(d(1:2));

        % angolo di questo vettore nel piano XY
        phi_i = atan2(d_xy(2), d_xy(1));

        % vogliamo che x_i = [cos λi, sin λi] punti verso phi_i
        % quindi l'ottimo è semplicemente:
        lambda_opt(i) = phi_i;
    end

end