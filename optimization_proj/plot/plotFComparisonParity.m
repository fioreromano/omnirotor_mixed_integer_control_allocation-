function plotFComparisonParity(Track, k, outputFolder)

ic_list = 1:size(Track.F,2);
clipPercentiles = [2 98];
ic0 = 1;                     % CoM di riferimento
f0  = Track.F(:,ic0);        % M×1 (129)
% colormap con abbastanza colori distinti
C = lines(numel(ic_list));

fig = figure('Color','w','Name','Parity plot: all CoM shifts vs ic0');
hold on; grid on;

allv = f0(:);

leg = {};

cc = 0;
for ii = 1:numel(ic_list)
    ic = ic_list(ii);
    if ic == ic0, continue; end

    cc = cc + 1;
    fi = Track.F(:,ic);
    fbest = min(fi); 
    scatter(f0, fi, 28, C(cc,:), 'filled', ...
        'MarkerFaceAlpha',0.6, 'MarkerEdgeAlpha',0);

    yline(fbest, '--', ...
        'Color', C(cc,:), ...
        'LineWidth', 1.5, ...
        'HandleVisibility','off');   % evita voci duplicate in legenda

    allv = [allv; fi(:)];
    leg{cc} = sprintf('shift = %.3g', Track.shift(ic));
end

% limiti robusti per assi
lo = prctile(allv, clipPercentiles(1));
hi = prctile(allv, clipPercentiles(2));

% limiti "stretti" sugli assi, basati sui dati reali
xlo = min(f0);
xhi = max(f0);

ylo = min(fi);
yhi = max(fi);

pad = 0.05 * max(xhi-xlo, yhi-ylo);   % piccolo margine visivo

xlim([xlo-pad, xhi+pad]);
ylim([lo, hi]);
xlabel(sprintf('f at standard configuration'));
ylabel('f at CoM shift');

legend([leg], 'Location','bestoutside');
fname = sprintf('%s/plotFcomparison_k%d.png', outputFolder, k);
exportgraphics(fig, fname, 'Resolution', 300);
end