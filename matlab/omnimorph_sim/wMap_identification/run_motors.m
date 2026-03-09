%% Generate equally spaced orientations
delZ = 2*pi/8;
delY = 2*pi/8; 
i = 1;

for thZ = 0:delZ:2*pi
    for thY = 0:delY:2*pi
        R(:,:,i) = Ry(thY) * Rz(thZ);
        i = i + 1;
    end
end

% Input allocation
desForceBody(:,1) = zeros(3,1);
k=1;
numForces = 1;

figure
hold on
view(3)
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');

origin = [0,0,0];
for j=1:numForces
    for i = 1:numel(R(1,1,:))
        % Compute thrust vectors to cancel gravity with varying magnitudes
        currentForce = R(:,:,i)' * [0; 0; (j/numForces)*config.uavParams.mass*config.gravity];
    
        if i > 1
            isDuplicate = any(all(abs(desForceBody(:,1:end) - currentForce) < 1e-6, 1));
            if isDuplicate
                continue;  % Skip duplicate
            end
        end
    
        % Do input allocation
        desForceBody(:,k) = currentForce;
        desWrench(:,k) = [currentForce; 0; 0; 0];
        inp(:,k) = pinv(config.uavParams.wrenchMap) * desWrench(:,k);
        propSpeeds(:,k) = sign(inp(:,k)) .* sqrt(abs(inp(:,k)));
        k = k+1;
    
        quiver3(origin(1),       origin(2),       origin(3), ...
                currentForce(1), currentForce(2), currentForce(3), ...
                0, 'LineWidth', 2, 'Color', 'b');
        drawnow
        pause(0.1)
    end
end

%% Generate random vectors in regions
numData = 5; % Amount of random vectors
k=1;

% Azimuth and elevation regions
reg = [-pi,      pi,     -pi,      pi,     pi/3,   pi/2;
     -pi/4,    pi/4,   -pi/4,    pi/4,   0,      pi/3;
     -3*pi/4, -pi/4,   -3*pi/4, -pi/4,   0,      pi/3;
      pi/4,    3*pi/4,  pi/4,    3*pi/4, 0,      pi/3;
     -pi,     -3*pi/4,  3*pi/4,  pi,     0,      pi/3;
     -pi,      pi,     -pi,      pi,     -pi/2, -pi/3;
     -pi/4,    pi/4,   -pi/4,    pi/4,   -pi/3,  0;
     -3*pi/4, -pi/4,   -3*pi/4, -pi/4,   -pi/3,  0;
      pi/4,    3*pi/4,  pi/4,    3*pi/4, -pi/3,  0;
     -pi,     -3*pi/4,  3*pi/4,  pi,     -pi/3,  0;];
regSelec = 1; % Select region 
colors = lines(numel(reg(:,1)));

% Plot thrust vectors
figure(1)
hold on
view(3)
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
origin = [0,0,0];

while true
    lb = 0.9;
    ub = 1.1;
    r = lb + (ub - lb) * rand(); % Random vector magnitude
        
    v = randn(3, 1);      % Sample from standard normal distribution
    v = v / norm(v);      % Normalize to unit length
    [az,ev,~] = cart2sph(v(1),v(2),v(3));
    
    % Check if vector is in desired region
    if ~(((az>reg(regSelec,1) && az<reg(regSelec,2)) || (az>reg(regSelec,3) && az<reg(regSelec,4))) && (ev>reg(regSelec,5) && ev<reg(regSelec,6)))
        continue
    end
    
    % Do input allocation
    currentForce = v * r * config.uavParams.mass * config.gravity;
    ix = k;
    desForceBody(:,ix) = currentForce;
    desWrench(:,ix) = [currentForce; 0; 0; 0];
    inp(:,ix) = pinv(config.uavParams.wrenchMap) * desWrench(:,ix);
    propSpeeds(:,ix) = sign(inp(:,ix)) .* sqrt(abs(inp(:,ix)));

    % Ensure no prop speeds in forbidden region
    if any(abs(propSpeeds(:,ix))<=30)
        continue
    end
        
    quiver3(origin(1),       origin(2),       origin(3), ...
            currentForce(1), currentForce(2), currentForce(3), ...
            0, 'LineWidth', 2, 'Color', colors(regSelec,:));
    drawnow
    pause(0.1)
    
    if k == numData
        break
    end
    k = k+1;
end

% Plot prop speeds
figure(2)
hold on
grid on
for j=1:numel(propSpeeds(1,:))
    plot(ones(1,8)*j,propSpeeds(:,j),'-o')
end

save(strcat(baseFolder,'propSpeeds.mat'), 'propSpeeds')

%% Run propellers - loop through propSpeeds
while true
    testChoice = input('Type I for individual motor, A for all motors, or q to quit: ', 's');

    if strcmpi(testChoice, 'q')
        disp('Exiting…');
        break
    end

    if strcmpi(testChoice, 'A')
        wrenchChoice = input(sprintf('Choose orientation to test (1-%s): ', num2str(numel(propSpeeds(1,:)))), 's');

        w = str2double(wrenchChoice);
        if isnan(w) || w < 1 || w > numel(propSpeeds(1,:)) || w ~= floor(w)
            fprintf('"%s" is not a valid integer between 1 and %s.\n', num2str(w), num2str(numel(propSpeeds(1,:))));
            continue          % restart loop and ask again
        end     

        rotorcraft.enable_motor(1);
        rotorcraft.enable_motor(2);
        rotorcraft.enable_motor(3);
        rotorcraft.enable_motor(4);
        rotorcraft.enable_motor(5);
        rotorcraft.enable_motor(6);
        rotorcraft.enable_motor(7);
        rotorcraft.enable_motor(8);

        rotorcraft.log(strcat(logPath,'rot_',num2str(w),'.log'));

        rotorcraft.start();
        pause(0.5)        

        rotorcraft.set_velocity(num2cell(0.2*propSpeeds(:,w)));
        pause(0.5)

        rotorcraft.set_velocity(num2cell(0.4*propSpeeds(:,w)));
        pause(0.5)    

        rotorcraft.set_velocity(num2cell(0.6*propSpeeds(:,w)));
        pause(0.5)           

        rotorcraft.set_velocity(num2cell(0.8*propSpeeds(:,w)));
        pause(0.5)          
    
        rotorcraft.set_velocity(num2cell(propSpeeds(:,w)));
        pause(4) 

        rotorcraft.set_velocity(num2cell([0,0,0,0,0,0,0,0]));
        pause(0.5)

        rotorcraft.log_stop();
        rotorcraft.stop();

        % Copy to local PC
        fprintf('Copying logs... \n');
        status = system(sprintf('scp %s@%s:%s %s', raspiUser, remoteIP, strcat(logPath,'rot_',num2str(w),'.log'), logFolder));
        if status == 0
            fprintf('Log files copied successfully.\n');
        else
            fprintf('Failed to copy log files.\n');
            fprintf('To copy log files from Raspi to local machine manually: \n')
            fprintf('scp %s@%s:%s %s \n', raspiUser, remoteIP, strcat(logPath,'rot_',num2str(w),'.log'), logFolder);        
        end
    end

    if strcmpi(testChoice, 'I')
        motChoice = input(sprintf('Select motor to test (1-8): '), 's');

        m = str2double(motChoice);
        if isnan(m) || m < 1 || m > 8 || m ~= floor(m)
            fprintf('"%s" is not a valid integer between 1 and 8.\n', m);
            continue          % restart loop and ask again
        end  

        rotorcraft.disable_motor(1);
        rotorcraft.disable_motor(2);
        rotorcraft.disable_motor(3);
        rotorcraft.disable_motor(4);
        rotorcraft.disable_motor(5);
        rotorcraft.disable_motor(6);
        rotorcraft.disable_motor(7);
        rotorcraft.disable_motor(8);

        rotorcraft.log(strcat(logPath,'rot_',num2str(m+numel(propSpeeds(1,:))),'.log'));
        
        e_i = zeros(1,8);   
        e_i(m) = 1;   
        
        rotorcraft.enable_motor(m);
        rotorcraft.start();
        
        for j=20:20:200
            rotorcraft.set_velocity(num2cell(e_i*j));
            pause(2)
        end

        rotorcraft.set_velocity(num2cell([0,0,0,0,0,0,0,0]))
        pause(0.5)
            
        for j=-20:-20:-200
            rotorcraft.set_velocity(num2cell(e_i*j));
            pause(2)
        end
        
        rotorcraft.disable_motor(m);

        rotorcraft.log_stop();
        rotorcraft.stop();

        % Copy to local PC
        fprintf('Copying logs... \n');
        status = system(sprintf('scp %s@%s:%s %s', raspiUser, remoteIP, strcat(logPath,'rot_',num2str(m+numel(propSpeeds(1,:))),'.log'), logFolder));
        if status == 0
            fprintf('Log files copied successfully.\n');
        else
            fprintf('Failed to copy log files.\n');
            fprintf('To copy log files from Raspi to local machine manually: \n')
            fprintf('scp %s@%s:%s %s \n', raspiUser, remoteIP, strcat(logPath,'rot_',num2str(m+numel(propSpeeds(1,:))),'.log'), logFolder);        
        end
    end  

    % Clean up 
    fprintf('Cleaning up... \n');
    status = system(sprintf('ssh %s@%s "rm %s*.log"', raspiUser, remoteIP, logPath));
    if status == 0
        fprintf('Genom3 logs deleted successfully.\n');
    else
        fprintf('Failed to delete logs.\n');
        fprintf('To delete logs from Raspi manually: \n')
        fprintf('ssh %s@%s "rm %s*.log" \n', raspiUser, remoteIP, logPath);  
    end
end
