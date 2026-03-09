function plotPropellerClustersXY(Results, ic)

    % --- NUOVO ORDINE DI VISUALIZZAZIONE ROTORI ---
    rotorOrder = [2, 1, 6, 5, 3, 4, 7, 8];
    n = 8;

    figure('Color','w','Name',sprintf('CoM #%d – Propeller Clusters (XY)',ic));
    % --- normali della soluzione ottima ---
    vbest   = Results(ic).best_v;
    xy_best = reshape(vbest, 2, n);
    N_best  = normals_from_xy(xy_best);

    % per legenda globale
    allLabels = [];

    for k = 1:8
        
        j = rotorOrder(k);   % propeller reale

        subplot(2,4,k); hold on; axis equal; grid on;

        % cerchio unitario (intersezione sfera con XY)
        th = linspace(0,2*pi,200);
        plot(cos(th), sin(th), 'k--','LineWidth',1);

        % dati cluster
        Nj   = Results(ic).clusters(j).points;
        lbls = Results(ic).clusters(j).labels;

        % proiezione XY
        Nx = Nj(:,1);
        Ny = Nj(:,2);

        scatter(Nx, Ny, 15, lbls, 'filled');

        xlabel('N_x'); ylabel('N_y');
        title(sprintf('Propeller %d', j));
        xlim([-1 1]); ylim([-1 1]);

        allLabels = [allLabels; lbls(:)];
        plot(N_best(1,j), N_best(2,j), ...
             'ko', 'MarkerSize', 9, 'LineWidth', 2);
    end

    % --- LEGENDA ---

    clusters = unique(allLabels);
    clusters(clusters <= 0) = [];   % rimuove noise DBSCAN

    nCl = numel(clusters);
    cmap = lines(nCl);
    colormap(cmap);

    h = gobjects(nCl+1,1);
    legTxt = cell(nCl+1,1);

    % handle cluster
    for i = 1:nCl
        h(i) = plot(nan,nan,'o', ...
            'MarkerFaceColor',cmap(i,:), ...
            'MarkerEdgeColor','k');
        legTxt{i} = sprintf('Cluster %d', clusters(i));
    end

    % handle soluzione ottima
    h(end) = plot(nan,nan,'ko', ...
        'MarkerSize',7,'LineWidth',2);
    legTxt{end} = 'Optimal solution';

    lgd = legend(h, legTxt, ...
        'Orientation','horizontal');

    % posizione centrata sotto
    lgd.Units = 'normalized';
    lgd.Position = [0.2 0.02 0.6 0.05];

end
