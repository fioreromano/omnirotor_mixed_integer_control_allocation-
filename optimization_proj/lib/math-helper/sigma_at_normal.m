function sigma = sigma_at_normal(d1, d2, n, u, v, ...
                                 Nopt, pi_B_now, j, k_sys)

    % perturbazione nel piano tangente locale
    delta = d1*u + d2*v;
    dnorm = norm(delta);

    if dnorm < 1e-12
        n1 = n;
    else
        tdir = delta / dnorm;
        n1 = cos(dnorm)*n + sin(dnorm)*tdir;
    end

    % sostituisci solo il propeller j
    Ntest = Nopt;
    Ntest(:,j) = n1;

    % matrice di allocazione
    if k_sys == 1
        B = AllocationMatrixBrescianini(Ntest, pi_B_now);
    else
        B = AllocationMatrixTilt(Ntest, pi_B_now);
    end

    % metrica
    sigma = min(svd(B));
end
