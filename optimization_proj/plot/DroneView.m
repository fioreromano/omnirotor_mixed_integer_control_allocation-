function [hCube, hOptPoint_top, hOptPoint_bottom, hOptLine, hCoM] = DroneView(P, N, center, L, a, b, showLegend)
    hold on;
    grid on;
    axis equal;

    n = size(P,2);
    
    % 1. Cube at platform vertices
    cube_vertices = P';
    edges = [
        1 2; 2 3; 3 4; 4 1;  
        5 6; 6 7; 7 8; 8 5;  
        1 5; 2 6; 3 7; 4 8   
    ];
    
    for i = 1:size(edges, 1)
        plot3(cube_vertices(edges(i,:), 1), cube_vertices(edges(i,:), 2), ...
              cube_vertices(edges(i,:), 3), 'Color', [0.7 0.7 0.7], ...
              'LineWidth', 0.5,'LineStyle', '--');
    end
    
    hCube = scatter3(cube_vertices(:,1), cube_vertices(:,2), cube_vertices(:,3), ...
             20, 'filled', 'MarkerFaceColor', [0.7 0.7 0.7]);

    % 2. Reference frame
    origin = center;
    axis_length = L*0.3;

    hFrameX = quiver3(origin(1), origin(2), origin(3), axis_length, 0, 0, ...
            'r', 'LineWidth', 0.5, 'MaxHeadSize', 0.5);
    hFrameY = quiver3(origin(1), origin(2), origin(3), 0, axis_length, 0, ...
            'g', 'LineWidth', 0.5, 'MaxHeadSize', 0.5);
    hFrameZ = quiver3(origin(1), origin(2), origin(3), 0, 0, axis_length, ...
            'b', 'LineWidth', 0.5, 'MaxHeadSize', 0.5);

    hCoM = scatter3(origin(1), origin(2), origin(3), ...
                45, '*', 'MarkerFaceColor','k', 'MarkerEdgeColor','k'); 


    text(axis_length, center(2), 0, '$x$', 'Interpreter', 'latex', 'FontSize', 7, 'Color', 'r', 'FontWeight', 'bold');
    text(0, axis_length + center(2), 0, '$y$', 'Interpreter', 'latex', 'FontSize', 7, 'Color', 'g', 'FontWeight', 'bold');
    text(0, center(2), axis_length, '$z$', 'Interpreter', 'latex', 'FontSize', 7, 'Color', 'b', 'FontWeight', 'bold');
    
    % 3. Per-propeller geometry
    for j = 1:n
        propeller_pos = cube_vertices(j,:);   
        
        % 4. Upper half-sphere
        sphere_radius = 0.05;
        [theta, phi] = meshgrid(linspace(0, 2*pi, 20), linspace(0, pi/2, 10));
        
        Xs = sphere_radius * sin(phi) .* cos(theta);
        Ys = sphere_radius * sin(phi) .* sin(theta);
        Zs = sphere_radius * cos(phi);
        
        Xs = Xs + propeller_pos(1);
        Ys = Ys + propeller_pos(2);
        Zs = Zs + propeller_pos(3);
        
        hCap = surf(Xs, Ys, Zs, 'FaceColor', [0.9 0.7 0.1], 'FaceAlpha', 0.3, ...
                    'EdgeColor', [0.7 0.5 0.1], 'EdgeAlpha', 0.2);
        
        % 5. Optimal point
        normal_opt = N(:,j);
        point_on_sphere = propeller_pos + normal_opt' * sphere_radius;
        z_val = point_on_sphere(3); 
        if j > 4
            % Propeller inferiori
            point_color = [1.0, 0.5, 0.0];  % Arancione fisso
            marker_type = '^';  
            marker_size = 30;  
        else
            point_color = [0.8, 0.1 , 0.1];
            % Propeller superiori 
            marker_type = 'o';  %
            marker_size = 30;
        end
        
        scatter3(point_on_sphere(1), point_on_sphere(2), point_on_sphere(3), ...
                             marker_size, 'filled', ...
                            'MarkerFaceColor', point_color, 'Marker', marker_type);

        hOptLine = plot3([propeller_pos(1), point_on_sphere(1)], ...
              [propeller_pos(2), point_on_sphere(2)], ...
              [propeller_pos(3), point_on_sphere(3)], ...
              'Color', 'b', 'LineWidth', 0.8);

        text(propeller_pos(1), propeller_pos(2), propeller_pos(3) + 0.08, ...
             sprintf('$P_{%d}$', j), 'Interpreter', 'latex', ...
             'FontSize', 10, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center');
    end

    
    % Axis labels
    xlabel('$x~[\mathrm{m}]$', 'Interpreter', 'latex', 'FontSize', 14);
    ylabel('$y~[\mathrm{m}]$', 'Interpreter', 'latex', 'FontSize', 14);
    zlabel('$z~[\mathrm{m}]$', 'Interpreter', 'latex', 'FontSize', 14);

    hOptPoint_top = scatter(nan, nan, 30, 'o', 'filled', 'MarkerFaceColor', [0.8,0.1,0.1]);
    hOptPoint_bottom = scatter(nan, nan, 30, '^', 'filled', 'MarkerFaceColor', [0.8,0.5,0.1]);

    view(a, b);
    set(gca, 'FontSize', 11);
    grid on;
    box on;

    if showLegend
    legend( ...
    [hCube, hOptPoint_top, hOptPoint_bottom, hOptLine, hCoM], ...
    { '$\mathrm{Propeller~Vertex}$', ...
      '$\mathrm{Optimized~Point~(top)}$', ...
      '$\mathrm{Optimized~Point~(bottom)}$', ...
      '$\mathrm{Optimized~Vector}$', ...
      '$\mathrm{CoM}$' }, ...
    'Interpreter','latex', ...
    'FontSize', 11, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
    end
    if nargout == 0
        clear hCube hOptPoint hOptLine hCoM
    end

end
