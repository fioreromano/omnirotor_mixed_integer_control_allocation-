function plotDroneConfiguration(P, CoM, CoG, N)
% PLOTDRONECONFIGURATION - Visualizza la configurazione 3D del drone
% Input:
%   P   - 8x3 positions of motors [x,y,z] (o 3x8 trasposto)
%   CoM - 1x3 or 3x1 Center of Mass
%   CoG - 1x3 or 3x1 Center of Gravity
%   N   - 3x8 matrix, each column is the normal (thrust direction) of a propeller
%
% Esempio:
%   plotDroneConfiguration(P, CoM, CoG, N)

% --- VALIDAZIONE INPUT (compatibilità trasposta)
if size(P,1) == 3 && size(P,2) == 8
    P = P'; % trasponi se necessario
end
assert(all(size(P) == [8 3]), 'P deve essere 8x3 (o 3x8 trasposto).');

if numel(CoM) == 3
    CoM = reshape(CoM,1,3);
else
    error('CoM deve essere vettore di 3 elementi.');
end
if numel(CoG) == 3
    CoG = reshape(CoG,1,3);
else
    error('CoG deve essere vettore di 3 elementi.');
end

% garantisci orientamento N = 3x8
if size(N,1) == 8 && size(N,2) == 3
    N = N'; % trasponi se necessario
end
assert(all(size(N) == [3 8]), 'N deve essere 3x8 (ogni colonna è una normale).');

% inizializza figura/assi
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

% Plot main points
h1 = plot3(P(:,1), P(:,2), P(:,3), 'ko', 'MarkerSize',3, 'MarkerFaceColor','b'); % motors
h2 = plot3(CoM(1), CoM(2), CoM(3), 'b*', 'MarkerSize',5,'LineWidth',1.5); % CoM
h3 = plot3(CoG(1), CoG(2), CoG(3), 'ks', 'MarkerSize',5,'LineWidth',1.5); % CoG

title('Drone 3D Visualization (from normals \bfN)', 'Interpreter','latex');

% Body frame axes at CoM
scale_body = 0.1 * max(range(P(:,1)), range(P(:,2))); % scala relativa alla size del drone
if scale_body == 0, scale_body = 0.05; end
quiver3(CoM(1), CoM(2), CoM(3), scale_body, 0, 0, 'r', 'LineWidth',1, 'MaxHeadSize',0.5);
quiver3(CoM(1), CoM(2), CoM(3), 0, scale_body, 0, 'g', 'LineWidth',1, 'MaxHeadSize',0.5);
quiver3(CoM(1), CoM(2), CoM(3), 0, 0, scale_body, 'b', 'LineWidth',1, 'MaxHeadSize',0.5);

% Draw cube bounding box (optional)
xmin = min(P(:,1)); xmax = max(P(:,1));
ymin = min(P(:,2)); ymax = max(P(:,2));
zmin = min(P(:,3)); zmax = max(P(:,3));

cubeVertices = [
    xmin ymin zmin;
    xmax ymin zmin;
    xmax ymax zmin;
    xmin ymax zmin;
    xmin ymin zmax;
    xmax ymin zmax;
    xmax ymax zmax;
    xmin ymax zmax
];

edges = [
    1 2; 2 3; 3 4; 4 1;
    5 6; 6 7; 7 8; 8 5;
    1 5; 2 6; 3 7; 4 8
];

for e = 1:size(edges,1)
    v1 = cubeVertices(edges(e,1),:);
    v2 = cubeVertices(edges(e,2),:);
    plot3([v1(1),v2(1)], [v1(2),v2(2)], [v1(3),v2(3)], '--', 'Color',[0.6 0.6 0.6]);
end

% ==== Compute local frames for each propeller from N ====
Z = zeros(3,8);
X = zeros(3,8);
Y = zeros(3,8);

for i = 1:8
    zi = N(:,i);
    if norm(zi) == 0
        warning('Normal %d is zero vector; skipping.', i);
        Z(:,i) = [0;0;1];
    else
        Z(:,i) = zi / norm(zi);
    end

    % pick a non-collinear vector
    if abs(dot(Z(:,i), [0;0;1])) < 0.9
        temp = [0;0;1];
    else
        temp = [1;0;0];
    end

    xi = cross(temp, Z(:,i));
    xi = xi / norm(xi);
    yi = cross(Z(:,i), xi);

    X(:,i) = xi;
    Y(:,i) = yi;
end

% ==== Plot propeller frames (quiver) ====
scale_prop = 0.1 * max([range(P(:,1)); range(P(:,2)); max(1e-3, range(P(:,3)))]);
for i = 1:8
    px = P(i,1); py = P(i,2); pz = P(i,3);

    quiver3(px, py, pz, scale_prop*X(1,i), scale_prop*X(2,i), scale_prop*X(3,i), ...
            'r', 'LineWidth', 1.2, 'MaxHeadSize', 0.5);
    quiver3(px, py, pz, scale_prop*Y(1,i), scale_prop*Y(2,i), scale_prop*Y(3,i), ...
            'g', 'LineWidth', 1.2, 'MaxHeadSize', 0.5);
    quiver3(px, py, pz, scale_prop*Z(1,i), scale_prop*Z(2,i), scale_prop*Z(3,i), ...
            'b', 'LineWidth', 1.2, 'MaxHeadSize', 0.5);
end

% ==== Draw circle perpendicular to thrust axis (normal Z_i) ====
R_circ = 0.015 * max([range(P(:,1)); range(P(:,2)); 1]); % scala automatica
theta = linspace(0, 2*pi, 180);

for i = 1:8
    px = P(i,1); py = P(i,2); pz = P(i,3);
    Xi = X(:,i);
    Yi = Y(:,i);

    circ = R_circ * (Xi*cos(theta) + Yi*sin(theta));
    xc = px + circ(1,:);
    yc = py + circ(2,:);
    zc = pz + circ(3,:);

    % Riempimento sottile + bordo
    fill3(xc, yc, zc, [0.92 0.92 0.92], 'EdgeColor','none', 'FaceAlpha', 0.5);
    plot3(xc, yc, zc, '-', 'Color',[0.2 0.2 0.2], 'LineWidth', 1.0);
end
legend([h1 h2 h3], ...
       {'Propellers','CoM','CoG'}, ...
       'Location','southoutside', ...
       'Orientation','horizontal');


set(gcf, 'Color', 'white');
view(3);
rotate3d on;

end
