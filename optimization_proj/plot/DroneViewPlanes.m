function [hCube, hPlane, hOptLine, hCoM] = DroneViewPlanes(P, N, center, a, b, showLegend)
    hold on;
    grid on;
    axis equal;
    n = size(P,2);
    num_sols  = size(N, 3);
    
    % --- 1. Geometria Frame (Cubo) ---
    cube_vertices = P';
    edges = [
        1 2; 2 3; 3 4; 4 1;  
        5 6; 6 7; 7 8; 8 5;  
        1 5; 2 6; 3 7; 4 8   
    ];
    
    for i = 1:size(edges, 1)
        plot3(cube_vertices(edges(i,:), 1), cube_vertices(edges(i,:), 2), ...
              cube_vertices(edges(i,:), 3), 'Color', [0.6 0.6 0.6], ...
              'LineWidth', 0.5,'LineStyle', '-');
    end
    
    hCube = scatter3(cube_vertices(:,1), cube_vertices(:,2), cube_vertices(:,3), ...
             20, 'filled', 'MarkerFaceColor', [0.5 0.5 0.5]);

    % --- 2. Sistema di Riferimento CoM ---
    origin = center;
    axis_length = 0.2*0.2;
    quiver3(origin(1), origin(2), origin(3), axis_length, 0, 0, 'r', 'LineWidth', 1);
    quiver3(origin(1), origin(2), origin(3), 0, axis_length, 0, 'g', 'LineWidth', 1);
    quiver3(origin(1), origin(2), origin(3), 0, 0, axis_length, 'b', 'LineWidth', 1);
    
    hCoM = scatter3(origin(1), origin(2), origin(3), ...
                10, 'o', 'filled', 'MarkerFaceColor','k', 'MarkerEdgeColor','k'); 

    % --- 3. Loop sui Propeller per Piani e Vettori ---
    plane_size = 0.05; % Dimensione visiva del piano (semi-lato)
    vec_scale  = 0.05; % Lunghezza vettore spinta
    
    hPlane = []; % Handle dummy per la legenda
    
    for j = 1:n
        prop_pos = P(:,j);   
        
        % A. Calcolo vettore Raggio (CoM -> Prop)
        r_vec = prop_pos - center;
        r_dir = r_vec / norm(r_vec);
        
        % B. Disegno la linea del raggio (opzionale, per chiarezza)
        plot3([center(1) prop_pos(1)], [center(2) prop_pos(2)], ...
              [center(3) prop_pos(3)], 'Color', [0.6 0.6 0.6], 'LineStyle', ':');
        
        % C. Costruzione Base del Piano Tangente (perpendicolare a r_dir)
        % Cerco un vettore u temporaneo non parallelo a r_dir
        if abs(dot(r_dir, [0;0;1])) > 0.9
            ref = [0;1;0];
        else
            ref = [0;0;1];
        end
        u_vec = cross(r_dir, ref);
        u_vec = u_vec / norm(u_vec);
        v_vec = cross(r_dir, u_vec); % v è ortogonale a r e u
        
        % D. Creazione coordinate del quadrato (Patch)
        % Angoli del quadrato rispetto al centro del prop
        c1 = prop_pos + plane_size * ( u_vec + v_vec);
        c2 = prop_pos + plane_size * (-u_vec + v_vec);
        c3 = prop_pos + plane_size * (-u_vec - v_vec);
        c4 = prop_pos + plane_size * ( u_vec - v_vec);
        
        Xp = [c1(1) c2(1) c3(1) c4(1)];
        Yp = [c1(2) c2(2) c3(2) c4(2)];
        Zp = [c1(3) c2(3) c3(3) c4(3)];
        
        % Plot del Piano
        hPlane = patch(Xp, Yp, Zp, 'c', 'FaceAlpha', 0.2, 'EdgeColor', 'c', 'EdgeAlpha', 0.5);
        
        % E. Vettore Ottimo
        % --- B. Disegna i Fasci di Vettori ---
    
        for j = 1:n
            prop_pos = P(:,j);
            
            % 1. Costruzione Array Unico per le linee sub-ottime
            % Usiamo NaN per staccare i segmenti in un unico comando plot3 (molto veloce)
            X_bundle = nan(3, num_sols); 
            Y_bundle = nan(3, num_sols);
            Z_bundle = nan(3, num_sols);
            
            % Estraiamo le normali per questo motore: 3 x Nsols
            normals_j = squeeze(N(:, j, :)); 
            
            % Coordinate Start (Propeller Pos)
            X_bundle(1,:) = prop_pos(1);
            Y_bundle(1,:) = prop_pos(2);
            Z_bundle(1,:) = prop_pos(3);
            
            % Coordinate End (Propeller Pos + Vector)
            X_bundle(2,:) = prop_pos(1) + normals_j(1,:) * vec_scale;
            Y_bundle(2,:) = prop_pos(2) + normals_j(2,:) * vec_scale;
            Z_bundle(2,:) = prop_pos(3) + normals_j(3,:) * vec_scale;
                        

            hOptAllLine = line(X_bundle(:), Y_bundle(:), Z_bundle(:), ...
                 'Color', [0, 0.4, 1, 0.15], 'LineWidth', 0.5); 
             
            % 2. Disegna la MIGLIORE soluzione (Freccia Rossa opaca)
            n_best = N(:, j, 1);
            n_best = n_best / norm(n_best); 
            
            hOptLine = quiver3(prop_pos(1), prop_pos(2), prop_pos(3), ...
                    n_best(1)*vec_scale, n_best(2)*vec_scale, n_best(3)*vec_scale, ...
                    'Color', 'r', 'LineWidth', 1, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
                
            % Label ID
            text(prop_pos(1), prop_pos(2), prop_pos(3)+0.05, sprintf('P%d', j), ...
                 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        end
    
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(a,b);
                       
    if showLegend
        legend([hCoM, hPlane, hOptAllLine, hOptLine], ...
               {'Center of Mass', 'Plane', 'SubOptimum', 'Optimum'}, ...
               'Location', 'best');
    end
    
    if nargout == 0
        clear hCube hPlane hOptAllLine hOptLine hCoM
    end
end