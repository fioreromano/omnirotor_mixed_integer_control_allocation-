function Slice = plotConditionalSlice_fromFmincon_perpPlane(ic, k, eps_norm, eps_obj)

Nphi = 720; 

if k == 1
    load("results_Brescianini.mat");
else
    load("results_Complete.mat");
end

n = size(pi_B_Bcom_all,2);
rotorOrder = [2 1 6 5 3 4 7 8];
% bracci rispetto al CoM (come nel tuo optimization)
Pcom = pi_B_Bcom_all(:,:,ic);

pi_cg = pi_B_Bcom_all(:,:,1);

% basi perpendicolari a pi_cg
[U,V,~] = buildPerpBases(pi_cg, eps_norm);

% riferimento: soluzione fmincon
Nref = N_all(:,:,ic);   % 3×n

% per ciascun rotore, costruisco anche la fase di riferimento nel piano
% (proietto Nref nel piano span{U,V})
a_ref = sum(U .* Nref, 1);    % 1×n
b_ref = sum(V .* Nref, 1);    % 1×n
phi_ref = atan2(b_ref, a_ref);
phi_ref = mod(phi_ref, 2*pi);

phi_grid = linspace(0,2*pi,Nphi);

Slice = struct();
Slice.phi_ref = phi_ref;
Slice.Nref = Nref;

figure('Color','w','Name',sprintf('Conditional slices (perp plane) – CoM #%d',ic), ...
       'Units','normalized','Position',[0.02 0.05 0.96 0.85]);
t = tiledlayout(2,4,'TileSpacing','compact','Padding','compact');

for kk = 1:n
    j = rotorOrder(kk);

    f_j  = nan(Nphi,1);
    xy_j = nan(Nphi,2);

    for a = 1:Nphi
        % copia della configurazione best
        Ntest = Nref;

        % varia SOLO il rotore j nel suo piano perpendicolare
        nj = cos(phi_grid(a))*U(:,j) + sin(phi_grid(a))*V(:,j);

        % opzione: forzo emisfero superiore
        if nj(3) < 0
            nj = -nj;
        end

        Ntest(:,j) = nj;

        % valuta B e f (coerente col manifold)
        if k == 1
            B = AllocationMatrixBrescianini(Ntest, Pcom);
        else
            B = AllocationMatrixTilt(Ntest, Pcom);
        end

        svals = svd(B);
        smin  = max(min(svals), eps_obj);
        smax  = max(svals);

        f_j(a)  = -smin + 10*(smax/smin);
        xy_j(a,:) = [nj(1), nj(2)];
    end

    % ---- plot XY sul cerchio ----
    nexttile; hold on; axis equal; grid on;
    th = linspace(0,2*pi,300);
    plot(cos(th), sin(th), 'k--','LineWidth',1);

    scatter(xy_j(:,1), xy_j(:,2), 18, f_j, 'filled', ...
            'MarkerFaceAlpha',0.9,'MarkerEdgeAlpha',0);

    % stella sul "best" (proiezione sul piano)
    [~, ibest] = min(abs(phi_grid - phi_ref(j)));
    plot(xy_j(ibest,1), xy_j(ibest,2), 'rp', ...
         'MarkerSize',14,'MarkerFaceColor','r');

    title(sprintf('Rotor %d (others fixed)', j));
    xlabel('N_x'); ylabel('N_y');
    xlim([-1 1]); ylim([-1 1]);

    Slice(j).phi_grid = phi_grid;
    Slice(j).f = f_j;
    Slice(j).xy = xy_j;
end

cb = colorbar; cb.Layout.Tile = 'east';
cb.Label.String = 'f (conditional slice)';
colormap(turbo);

sgtitle(sprintf('Conditional slices from fmincon best – CoM #%d', ic));
end