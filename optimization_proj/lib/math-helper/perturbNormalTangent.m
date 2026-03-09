function n = perturbNormalTangent(n0, alpha)
%PERTURBNORMALTANGENT  Perturba una normale unitaria sull'emisfero
%   n0    : normale ottima (3x1, ||n0|| = 1)
%   alpha : massimo angolo di perturbazione [rad]

    % --- base ortonormale del piano tangente ---
    if abs(n0(3)) < 0.9
        a = [0; 0; 1];
    else
        a = [1; 0; 0];
    end

    t1 = cross(n0, a);
    t1 = t1 / norm(t1);

    t2 = cross(n0, t1);

    % --- campionamento isotropo nel disco tangente ---
    r     = alpha * sqrt(rand);   % distribuzione uniforme in area
    theta = 2*pi*rand;

    delta = r * (cos(theta)*t1 + sin(theta)*t2);

    % --- mappa esponenziale sulla sfera ---
    d = norm(delta);

    if d < 1e-12
        n = n0;
    else
        n = cos(d)*n0 + sin(d)*(delta/d);
    end
end
