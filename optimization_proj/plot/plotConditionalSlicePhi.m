function Slice = plotConditionalSlicePhi(ManifoldF, pi_B_Bcom_all, ic, k, eps_norm, eps_obj, outputFolder)

    Nphi = 720; 

    n = size(pi_B_Bcom_all,2);
    rotorOrder = [2, 1, 6, 5, 3, 4, 7, 8];

    Pcom = pi_B_Bcom_all(:,:,ic);              
    pi_cg = pi_B_Bcom_all(:,:,1);                        

    [U,V,~] = buildPerpBases(pi_cg, eps_norm);

    phi_ref = ManifoldF(k).CoM(ic).best.phi;          % 1×n

    phi_grid = linspace(0, pi, Nphi);

    Slice = struct();
    Slice(ic).phi_ref = phi_ref;

    fig1 = figure('Color','w','Name',sprintf('Conditional slices – CoM #%d',ic), ...
           'Units','normalized','Position',[0.02 0.05 0.96 0.85]);
    t = tiledlayout(2,4,'TileSpacing','compact','Padding','compact');

    for kk = 1:n
        j = rotorOrder(kk);
        f_j   = nan(Nphi,1);
        xy_j  = nan(Nphi,2);
        smin_j = nan(Nphi,1);
        cond_j = nan(Nphi,1);

        for a = 1:Nphi
            phi = phi_ref;
            phi(j) = phi_grid(a);

            % costruisci Ntest sul manifold
            Ntest = zeros(3,n);
            for r = 1:n
                nr = cos(phi(r))*U(:,r) + sin(phi(r))*V(:,r);
                if nr(3) < 0
                    nr = -nr;
                end
                Ntest(:,r) = nr;
            end

            % valutazione B
            if k == 1
                B = AllocationMatrixNoDrag(Ntest, Pcom);
            else
                B = AllocationMatrixComplete(Ntest, Pcom);
            end

            svals = svd(B);
            smin  = max(min(svals), eps_obj);
            smax  = max(svals);

            f_j(a) = -smin + 10*(smax/smin);
            smin_j(a) = smin;
            cond_j(a) = smax/smin;

            xy_j(a,:) = [Ntest(1,j), Ntest(2,j)];
        end

        nexttile; hold on; axis equal; grid on;
        th = linspace(0,2*pi,300);
        plot(cos(th), sin(th), 'k--','LineWidth',1);

        scatter(xy_j(:,1), xy_j(:,2), 18, f_j, 'filled', ...
                'MarkerFaceAlpha',0.9,'MarkerEdgeAlpha',0);

        [~, ibest] = min(abs(phi_grid - phi_ref(j)));
        
        plot(xy_j(ibest,1), xy_j(ibest,2), ...
             'rp', 'MarkerSize', 12, 'MarkerFaceColor','r');

        title(sprintf('Rotor %d (others fixed)', j));
        xlabel('N_x'); ylabel('N_y');
        xlim([-1 1]); ylim([-1 1]);

        % salva
        Slice(j).phi_grid = phi_grid;
        Slice(j).f = f_j;
        Slice(j).xy = xy_j;
        Slice(j).sigma_min = smin_j;
        Slice(j).cond = cond_j;
    end

    cb = colorbar;
    cb.Layout.Tile = 'east';
    cb.Label.String = 'f (conditional slice)';
    colormap(turbo);
    sgtitle(sprintf('CoM #%d', ic));


    fname1 = sprintf('%s/ConditionalSliceXY_CoM%d_k%d.png', outputFolder, ic, k);
    fname2 = sprintf('%s/ConditionalSlicePhi_CoM%d_k%d.png', outputFolder, ic, k);

    fig2 = figure('Color','w','Name',sprintf('f vs phi_j – CoM #%d',ic), ...
           'Units','normalized','Position',[0.02 0.05 0.96 0.85]);
    t2 = tiledlayout(2,4,'TileSpacing','compact','Padding','compact');

    for kk = 1:n
        j = rotorOrder(kk);
        nexttile; grid on; hold on;
        plot(Slice(j).phi_grid, Slice(j).f, 'LineWidth',1.2);
        xlabel('\phi_j [rad]'); ylabel('f');
        title(sprintf('Rotor %d', j));

        xline(phi_ref(j), '--', 'LineWidth', 1);
    end

    sgtitle(sprintf('Conditional slices: vary one rotor, others fixed at best – CoM #%d', ic));
    %exportgraphics(fig1, fname1, 'Resolution', 300);
    %exportgraphics(fig2, fname2, 'Resolution', 300);
end
