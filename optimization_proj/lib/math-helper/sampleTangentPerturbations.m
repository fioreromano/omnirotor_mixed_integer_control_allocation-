function [D, N] = sampleTangentPerturbations(n0, alpha, Nsamples)
%SAMPLETANGENTPERTURBATIONS
%   n0       : normale ottima (3x1)
%   alpha    : raggio angolare [rad]
%   Nsamples : numero campioni
%
%   D : Nsamples x 2  (coordinate nel piano tangente)
%   N : Nsamples x 3  (normali perturbate sull'emisfero)

    % base piano tangente
    if abs(n0(3)) < 0.9
        a = [0; 0; 1];
    else
        a = [1; 0; 0];
    end

    t1 = cross(n0, a); t1 = t1 / norm(t1);
    t2 = cross(n0, t1);

    D = zeros(Nsamples,2);
    N = zeros(Nsamples,3);

    for k = 1:Nsamples
        r     = alpha * sqrt(rand);
        theta = 2*pi*rand;

        delta = r*(cos(theta)*t1 + sin(theta)*t2);
        D(k,:) = [r*cos(theta), r*sin(theta)];

        d = norm(delta);
        if d < 1e-12
            n = n0;
        else
            n = cos(d)*n0 + sin(d)*(delta/d);
        end

        N(k,:) = n';
    end
end
