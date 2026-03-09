function DroneFullPlot(P, N, com, L)

figure; hold on; axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Drone 3D View with Rotor Disks');

% ---------------------------------------------------------------------
% DRONE CUBE (struttura leggera)
% ---------------------------------------------------------------------
% Ottieni i limiti del cubo dal bounding box dei propeller
minP = min(P,[],2); 
maxP = max(P,[],2);

% Vertici del cubo
cubeX = [minP(1) maxP(1) maxP(1) minP(1) minP(1) maxP(1) maxP(1) minP(1)];
cubeY = [minP(2) minP(2) maxP(2) maxP(2) minP(2) minP(2) maxP(2) maxP(2)];
cubeZ = [minP(3) minP(3) minP(3) minP(3) maxP(3) maxP(3) maxP(3) maxP(3)];

edges = [
        1 2; 2 3; 3 4; 4 1;  
        5 6; 6 7; 7 8; 8 5;  
        1 5; 2 6; 3 7; 4 8   
    ];

for e = 1:size(edges,1)
    plot3([cubeX(edges(e,1)) cubeX(edges(e,2))], ...
          [cubeY(edges(e,1)) cubeY(edges(e,2))], ...
          [cubeZ(edges(e,1)) cubeZ(edges(e,2))], ...
          'Color',[0.5 0.5 0.5],'LineWidth',0.8);
end

% ---------------------------------------------------------------------
% Disegna propeller points
% ---------------------------------------------------------------------
plot3(P(1,:), P(2,:), P(3,:), 'ko', 'MarkerSize', 8, 'MarkerFaceColor','k');

% ---------------------------------------------------------------------
% Disegna centro di massa
% ---------------------------------------------------------------------
plot3(com(1), com(2), com(3), 'ro', 'MarkerSize',10, 'LineWidth',1.5);

% ---------------------------------------------------------------------
% Disegna normali e rotor disks
% ---------------------------------------------------------------------
for i = 1:size(P,2)
    center = P(:,i);
    normal = N(:,i);

    % Disegna vettore normale
    quiver3(center(1), center(2), center(3), ...
            normal(1), normal(2), normal(3), ...
            0.15*L, 'Color',[0 0.2 1], 'LineWidth',1.4);

    % Disegna il rotor disk
    plotRotorDisk(center, normal, L);
end

view(45,20);
end
