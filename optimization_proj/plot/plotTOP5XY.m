function plotTOP5XY(Results_good, ic)

    rotorOrder = [2, 1, 6, 5, 3, 4, 7, 8];
    n = 8;

    figure('Color','w','Name',sprintf('CoM #%d – Propeller Results (XY)',ic));

    % ===== soluzione migliore =====
    [~, ibest] = min(Results_good(ic).fval);
    vbest = Results_good(ic).v(:, ibest);
    
    xy_best = reshape(vbest, 2, n);
    N_best  = normals_from_xy(xy_best);

    V = Results_good(ic).v;
    Ns = size(V,2);   
    C = lines(Ns);    

    % ===== handle legenda =====
    h_points = gobjects(1,1);
    h_best   = gobjects(1,1);
    h_circle = gobjects(1,1);
    firstScatter = true;


    for k = 1:8
        j = rotorOrder(k);

        subplot(2,4,k); hold on; axis equal; grid on;

        % cerchio unitario
        th = linspace(0,2*pi,200);
        h_circle = plot(cos(th), sin(th), ...
            'k--','LineWidth',1);

        % dati Top 5%
        idx = (2*j-1):(2*j);
        XY  = V(idx, :);

        if firstScatter
            h_points = scatter(XY(1,:), XY(2,:), ...
                20, C, 'filled', ...
                'MarkerFaceAlpha', 0.7);
            firstScatter = false;
        else
            scatter(XY(1,:), XY(2,:), ...
                20, C, 'filled', ...
                'MarkerFaceAlpha', 0.7, ...
                'HandleVisibility','off');
        end


        % soluzione ottima
        h_best = plot(N_best(1,j), N_best(2,j), ...
             'bo', 'MarkerSize',8, 'LineWidth',2.5);

        xlabel('N_x'); ylabel('N_y');
        title(sprintf('Propeller %d', j));
        xlim([-1 1]); ylim([-1 1]);
    end

    % ===== LEGENDA GLOBALE =====
    lgd = legend([h_points, h_best, h_circle], ...
        {'Top 0.1% solutions', 'Optimal solution', 'Unit circle'}, ...
        'Orientation','horizontal');

    lgd.Units = 'normalized';
    lgd.Position = [0.25 0.02 0.5 0.05];

    sgtitle(sprintf('XY projection of propeller normals – CoM #%d (Top 5%%)', ic));
end
