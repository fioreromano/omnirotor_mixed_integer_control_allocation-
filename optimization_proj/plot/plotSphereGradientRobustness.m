function plotSphereGradientRobustness(SphereMap, ic, k, outputFolder)

rotorOrder = [2, 1, 6, 5, 3, 4, 7, 8];
nProp = numel(rotorOrder);

figure('Color','w', ...
    'Name', sprintf('Robustness - CoM %d', ic), ...
    'Units','normalized', ...
    'Position',[0.05 0.05 0.75 0.5]);

t = tiledlayout(2,4, ...
    'TileSpacing','compact', ...
    'Padding','compact');

for k1 = 1:nProp
    j = rotorOrder(k1);
    nexttile; hold on;
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');

    % --- Data ---
    X = SphereMap(ic,j).X;
    Y = SphereMap(ic,j).Y;
    Z = SphereMap(ic,j).Z;
    G = SphereMap(ic,j).Grad.Norm;
    n0 = SphereMap(ic,j).n0;
    D = SphereMap(ic,j).SigmaMap;     % Δσ = σ - σ0
    v = D(~isnan(D));   

    % --- Gradient magnitude surface ---
    surf(X, Y, Z, D, ...
        'EdgeColor','none');

    % --- Reference hemisphere ---
    [xs,ys,zs] = sphere(40);
    zs(zs < 0) = NaN;
    surf(xs,ys,zs, ...
        'FaceAlpha',0.1, ...
        'EdgeColor','none');
    thetaMax = deg2rad(10);
    r = sin(thetaMax) *1.7;  % margine visivo
    
    xlim(n0(1) + r*[-1 1]);
    ylim(n0(2) + r*[-1 1]);
    zlim(n0(3) + r*[-1 1]);

    % --- Optimal normal ---
    plot3(n0(1),n0(2),n0(3), ...
        'ko','MarkerFaceColor','k','MarkerSize',2);

    title(sprintf('\\textbf{Propeller %d}', j), ...
        'Interpreter','latex','FontSize',11);

    view(35,30);
    colormap(parula);
    lo = prctile(v, 1);   
    hi = 0;               
    caxis([lo hi]);
end

% --- Shared colorbar ---
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Label.String = '$(\sigma - \sigma^*)$';

cb.Label.Interpreter = 'latex';
cb.Label.FontSize = 16;
cb.FontSize = 12;

% --- Global title ---
sgtitle(sprintf('Robustness landscape - CoM %d', ic), ...
    'Interpreter','latex','FontSize',16);

% --- Optional save ---
if nargin > 3 && ~isempty(outputFolder)
    filename = fullfile(outputFolder, ...
        sprintf('LocalRobustness3D_CoM_%d_%d.pdf', ic, k));
    W = 24; H = 14; m = 1.2;
    set(gcf,'Units','centimeters');
    set(gcf,'PaperUnits','centimeters', ...
        'PaperSize',[W H], ...
        'PaperPosition',[m m W-2*m H-2*m]);
    print(gcf, filename, '-dpdf','-painters');
    close(gcf);
end

end
