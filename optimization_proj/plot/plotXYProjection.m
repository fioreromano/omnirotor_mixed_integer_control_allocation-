function plotXYProjection(N, showLegend)
    hold on;
    grid on;
    axis equal;
    elevation_deg = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90];  % In gradi
    elevation_rad = deg2rad(elevation_deg);        % Converti in radianti
    
    % Calcola i raggi corrispondenti a queste elevazioni
    % Per una sfera unitaria: z = cos(elevazione), r = sin(elevazione)
    radii = sin(elevation_rad);
    
    theta_circ = linspace(0, 2*pi, 100);
    n = 8;
    
    for i = 1:length(radii)
        r = radii(i);
        elevation = elevation_deg(i);  % Ottieni l'elevazione in gradi
        
        % Calcola il colore in base all'elevazione
        % Più scuro al centro (elevazione bassa), più chiaro al bordo
        gray_intensity = 0.9 - 0.3 * (elevation/90);
        fill_color = [gray_intensity, gray_intensity, gray_intensity];
        
        if i == 1
            % Anello centrale
            fill(r*cos(theta_circ), r*sin(theta_circ), fill_color, ...
                 'FaceAlpha', 0.5, 'EdgeColor', fill_color*0.8, ...
                 'LineWidth', 1, 'LineStyle', '-');
        else
            % Anelli concentrici
            r_prev = radii(i-1);
            theta_fill = [theta_circ, fliplr(theta_circ)];
            x_fill = [r_prev*cos(theta_circ), r*cos(fliplr(theta_circ))];
            y_fill = [r_prev*sin(theta_circ), r*sin(fliplr(theta_circ))];
            
            fill(x_fill, y_fill, fill_color, ...
                 'FaceAlpha', 0.5, 'EdgeColor', fill_color*0.8, ...
                 'LineWidth', 1, 'LineStyle', '-');
        end
        
        % MODIFICA: etichetta con i gradi invece del valore z
        % Posiziona le etichette lungo l'asse x positivo
        text(r + 0.005, 0.005, sprintf('$%d^\\circ$', elevation), ...
            'Interpreter', 'latex', ...
            'FontSize', 7, 'Color', 'k', ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
    end
    
    % MODIFICA AGGIUNTIVA: aggiungi linee tratteggiate per gli angoli principali
    angoli_principali = [0, 45, 90, 135, 180, 225, 270, 315];
    for angolo = angoli_principali
        x_end = 1.1 * cosd(angolo);
        y_end = 1.1 * sind(angolo);
       plot([0, x_end], [0, y_end], ':', 'LineWidth', 0.5, ...
        'Color', [0.5, 0.5, 0.5, 0.3]);
    end
    hOptLine = [];

    % Disegna proiezioni delle normali
    for j = 1:n
        normal = N(:, j);
        normal_xy = normal(1:2);
        z = normal(3);
        
        % Calcola l'elevazione di questo punto in gradi
        elevation_point = rad2deg(acos(z));
        
        if j > 4
            % Propeller inferiori - triangolo arancione
            point_color = [1.0, 0.5, 0.0];  % Arancione fisso
            marker_type = '^';  % Triangolo
            marker_size = 65;
        else            
            % Propeller superiori - stile originale (cerchio blu)
            point_color = [0.8, 0.1, 0.1];
            marker_type = 'o';  % Cerchio
            marker_size = 60;
        end
        
        % Disegna freccia per tutte le normali (blu per tutti)
        h = quiver(0, 0, normal_xy(1), normal_xy(2), 'b', 'LineWidth', 1, ...
               'MaxHeadSize', 0.3);
        if isempty(hOptLine)
            hOptLine = h;
        end
        
        % Disegna punto con marker appropriato
        scatter(normal_xy(1), normal_xy(2), marker_size, 'filled', ...
                'MarkerFaceColor', point_color, 'Marker', marker_type);
        
    end

    hOptPoint_top = scatter(nan, nan, 30, 'o', 'filled', 'MarkerFaceColor', [0.8,0.1,0.1]);
    hOptPoint_bottom = scatter(nan, nan, 30, '^', 'filled', 'MarkerFaceColor', [0.8,0.5,0.1]);

    % Assi
    xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 12);

    xlim([-1.3, 1.3]);
    ylim([-1.3, 1.3]);

    if showLegend
    lg = legend( ...
        [hOptPoint_top, hOptPoint_bottom, hOptLine], ...
        { '$\mathrm{Optimized~Point~(top)}$', ...
          '$\mathrm{Optimized~Point~(bottom)}$', ...
          '$\mathrm{Optimized~Vector}$'},...
        'Interpreter','latex', ...
        'FontSize', 11, ...
        'Location','southoutside', ...
        'Orientation','horizontal');
    
    end
   
end