%% Generate random vectors in regions
saveFolder = '/home/rveenstra/MTP/data/';

numData = 5; % Amount of random vectors
k = 1;

% Azimuth and elevation regions
reg = [-pi,      pi,     -pi,      pi,     deg2rad(65),   pi/2;
     -deg2rad(40),    deg2rad(40),   -deg2rad(40),    deg2rad(40),   deg2rad(20),      deg2rad(55);
     -deg2rad(120), -deg2rad(60),   -deg2rad(120), -deg2rad(60),   deg2rad(35),      deg2rad(55);
      deg2rad(55),    deg2rad(115),  deg2rad(55), deg2rad(115), deg2rad(35),      deg2rad(55);
     -pi,     -3*pi/4,  3*pi/4,  pi,     0,      pi/3;
     -pi,      pi,     -pi,      pi,     -pi/2, -pi/3;
     -pi/4,    pi/4,   -pi/4,    pi/4,   -pi/3,  0;
     -3*pi/4, -pi/4,   -3*pi/4, -pi/4,   -pi/3,  0;
      pi/4,    3*pi/4,  pi/4,    3*pi/4, -pi/3,  0;
     -pi,     -3*pi/4,  3*pi/4,  pi,     -pi/3,  0;];
regSelec = 3; % Select region 
colors = lines(numel(reg(:,1)));

figure(1)
hold on
view(3)
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
origin = [0,0,0];

while true 
    R = quat2rotm(randrot); % Generate random orientaiton
    v = R' * [0;0;1];
    v = v / norm(v);      % Normalize to unit length
    [az,ev,~] = cart2sph(v(1),v(2),v(3));
    
    % Check if vector is in desired region
    if ~(((az>reg(regSelec,1) && az<reg(regSelec,2)) || (az>reg(regSelec,3) && az<reg(regSelec,4))) && (ev>reg(regSelec,5) && ev<reg(regSelec,6)))
        continue
    end

    % Check propspeeds
    currentForce = R' * [0; 0; config.uavParams.mass*config.gravity];
    desWrench = [currentForce; 0; 0; 0];
    inp = pinv(config.uavParams.wrenchMap) * desWrench;
    propSpeeds = sign(inp) .* sqrt(abs(inp));    

    % Ensure no prop speeds in forbidden region
    if any(abs(propSpeeds)<50)
        continue
    end
   
    % Add orientation to Rots
    Rots(:,:,k) = R;
    quiver3(origin(1), origin(2), origin(3), ...
            v(1),      v(2),      v(3), ...
            0, 'LineWidth', 2, 'Color', colors(regSelec,:));
    drawnow
    pause(0.1)
    
    if k == numData
        break
    end
    k = k+1;
end

save(strcat(saveFolder, 'Rots.mat'), 'Rots')

%% Plot regions
reg = [-pi,      pi,     -pi,      pi,     pi/3,   pi/2;
       -pi/4,    pi/4,   -pi/4,    pi/4,   0,      pi/3;
       -3*pi/4, -pi/4,   -3*pi/4, -pi/4,   0,      pi/3;
        pi/4,    3*pi/4,  pi/4,    3*pi/4, 0,      pi/3;
       -pi,     -3*pi/4,  3*pi/4,  pi,     0,      pi/3;
       -pi,      pi,     -pi,      pi,     -pi/2, -pi/3;
       -pi/4,    pi/4,   -pi/4,    pi/4,   -pi/3,  0;
       -3*pi/4, -pi/4,   -3*pi/4, -pi/4,   -pi/3,  0;
        pi/4,    3*pi/4,  pi/4,    3*pi/4, -pi/3,  0;
       -pi,     -3*pi/4,  3*pi/4,  pi,     -pi/3,  0];

% Plot base sphere
n = 40;
[theta,phi] = meshgrid(linspace(0,pi,n), linspace(-pi,pi,n));
x = sin(theta).*cos(phi);
y = sin(theta).*sin(phi);
z = cos(theta);

figure
surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none'); 
colormap gray
hold on
axis equal
xlabel('x'); ylabel('y'); zlabel('z')


% Plot regions
for k = 1:size(reg,1)
    % Take azimuth/elevation bounds
    az1 = reg(k,1); az2 = reg(k,2);
    el1 = reg(k,5); el2 = reg(k,6);

    % Make a grid for this patch
    [az,el] = meshgrid(linspace(az1,az2,40), linspace(el1,el2,40));

    % Convert to Cartesian
    xp = cos(el).*cos(az);
    yp = cos(el).*sin(az);
    zp = sin(el);

    % Plot patch with random colour
    surf(xp,yp,zp,'FaceAlpha',0.8,'EdgeColor','none', ...
         'FaceColor',rand(1,3));
end

title('S^2 Sphere with wrench map regions')