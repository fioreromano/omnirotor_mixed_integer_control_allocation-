function DroneMultiView(P, N, com, L)

dx = com(1); dy = com(2); dz = com(3);
t = tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
% ===== Title del layout principale =====
title(t, sprintf('Optimal Drone Configuration (CoM Shift: dx = %.3f, dy = %.3f, dz = %.3f)', ...
        dx, dy, dz), ...
        'FontSize', 16, 'Interpreter','latex');

margin = 0.1;
xmin = min(P(1,:)) - margin; xmax = max(P(1,:)) + margin;
ymin = min(P(2,:)) - margin; ymax = max(P(2,:)) + margin;
zmin = min(P(3,:)) - margin; zmax = max(P(3,:)) + margin;

% ===== Tile 1: Drone 3D =====
ax1 = nexttile(t);
[hCube, hOptPoint_top, hOptPoint_bottom, hOptLine, hCoM] = DroneView(P, N, com, L, 0, 90, false);
title(ax1,'Top View','Interpreter','latex','FontSize',12);
axis(ax1,'equal');
xlim(ax1,[-0.3 0.3]); ylim(ax1,[-0.3 0.3]); zlim(ax1,[zmin zmax]);

% ===== Tile 2: Collapsed View =====
ax2 = nexttile(t);
plotXYProjection(N, false);
title(ax2,'Top View (Collapsed)','Interpreter','latex','FontSize',12);

% ======= Global legend =======
lg = legend( ...
    [hCube, hOptPoint_top, hOptPoint_bottom, hOptLine, hCoM], ...
    { '$\mathrm{Propeller~Vertex}$', ...
      '$\mathrm{Optimized~Point~(top)}$', ...
      '$\mathrm{Optimized~Point~(bottom)}$', ...
      '$\mathrm{Optimized~Vector}$', ...
      '$\mathrm{CoM}$' }, ...
    'Interpreter','latex', ...
    'FontSize', 11, ...
    'Orientation','horizontal');
lg.Layout.Tile = 'south';
lg.Layout.TileSpan = [1 2];

end
