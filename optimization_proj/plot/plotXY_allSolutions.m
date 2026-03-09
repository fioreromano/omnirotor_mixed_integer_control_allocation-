function plotXY_allSolutions(N_all, CoM_all)

numTests = size(N_all, 3);
    
    elevation_deg = 0:10:90;
    elevation_rad = deg2rad(elevation_deg);
    radii = sin(elevation_rad);     
    theta_circ = linspace(0, 2*pi, 200);

    baseColors = [
        0.90 0.10 0.10;   % rosso
        0.10 0.40 0.90;   % blu
        0.10 0.75 0.25;   % verde
        0.90 0.60 0.00;   % arancione
        0.60 0.10 0.75;   % viola
        0.00 0.70 0.70;   % ciano/teal
    ];
    
    brightColors = baseColors( mod(0:numTests-1, size(baseColors,1)) + 1 , : );

    markers = {'o', 's', '^', 'v', 'd', 'p', 'h', '*', 'x', '+'};
    if numTests > length(markers)
        markers = repmat(markers, 1, ceil(numTests/length(markers)));
        markers = markers(1:numTests);
    end
    
    % Dimensioni dei marker
    markerSizes = [18, 16, 20, 20, 20, 22, 22, 24, 16, 16];

    rotorOrder = [2, 1, 6, 5, 3, 4, 7, 8];

    for idx = 1:8
        rotor = rotorOrder(idx);
        subplot(2,4,idx); 
        hold on; axis equal; grid on;

        % -----------------------------------------------
        %     DISEGNO DELLE CURVE DI LIVELLO (ELEVATION)
        % -----------------------------------------------
        for i = 1:length(radii)
            r = radii(i);
            elev = elevation_deg(i);

            % colore più chiaro con elevazione maggiore
            gray_intensity = 0.9 - 0.3*(elev/90);
            col = [gray_intensity gray_intensity gray_intensity];

            if i == 1
                fill(r*cos(theta_circ), r*sin(theta_circ), col, ...
                     'FaceAlpha', 0.5, 'EdgeColor', col*0.8, 'LineWidth', 1);
            else
                r_prev = radii(i-1);
                x_fill = [r_prev*cos(theta_circ), r*cos(fliplr(theta_circ))];
                y_fill = [r_prev*sin(theta_circ), r*sin(fliplr(theta_circ))];

                fill(x_fill, y_fill, col, ...
                     'FaceAlpha', 0.5, 'EdgeColor', col*0.8, 'LineWidth', 0.5);
            end

            % Etichette in gradi (come richiesto)
            text(r + 0.02, 0.02, sprintf('$%d^\\circ$', elev), ...
                'Interpreter','latex', 'FontSize', 5, 'Color','k');
        end

        % -----------------------------------------------
        %     LINEE DIREZIONALI (0°, 45°, ..., 315°)
        % -----------------------------------------------
        main_angles = 0:45:315;
        for ang = main_angles
            plot([0, 1.1*cosd(ang)], [0, 1.1*sind(ang)], ':', ...
                 'LineWidth', 0.4, 'Color', [0.5 0.5 0.5 0.4]);
        end

        % -----------------------------------------------
        %     SCATTER DELLE PROIEZIONI OTTIME
        % -----------------------------------------------
        for k = 1:numTests
            N_k = N_all(:, rotor, k);
            xy = N_k(1:2);

           scatter(xy(1), xy(2), markerSizes(k), 'filled', ...
                'Marker', markers{k}, ...
                'MarkerFaceColor', brightColors(k,:), ...
                'MarkerEdgeColor', 'k');
        end

        xlabel('$x$', 'Interpreter','latex');
        ylabel('$y$', 'Interpreter','latex');

        xlim([-1.3, 1.3]); 
        ylim([-1.3, 1.3]);

        title(sprintf('Rotor %d', rotor), 'FontSize', 12);
        
        hold off;
    end


    axes('Position',[0 0 1 1],'Visible','off'); hold on;

    h_legend = gobjects(numTests,1);
    legendLabels = cell(numTests,1);

    for k = 1:numTests
        h_legend(k) = plot(nan,nan, markers{k}, ...
            'MarkerFaceColor', brightColors(k,:), ...
            'MarkerEdgeColor', 'k', ...
            'MarkerSize', 8);
        legendLabels{k} = sprintf('CoM shift = %.4f m', CoM_all(2,k));
    end

    legend(h_legend, legendLabels, ...
        'Interpreter','latex', ...
        'FontSize', 10, ...
        'NumColumns', 3, ...
        'Location','southoutside');

    sgtitle('Optimal XY Projections for All CoM Shifts', ...
        'Interpreter','latex','FontSize',14);
end
