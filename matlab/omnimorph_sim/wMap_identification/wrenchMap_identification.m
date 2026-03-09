%% Extract all data points from data folder
U = [];
Z = [];
W = [];

while true 
    baseFolder = uigetdir(pwd, 'Select folder containing data files');
    if isequal(baseFolder, 0)
        error('No folder selected. Aborting log extraction.');
    end
    % Ensure trailing slash
    if baseFolder(end) ~= filesep
        baseFolder = [baseFolder filesep];
    end

    trainDatFolder = strcat(baseFolder,'data/');

    % Import data
    fileList = dir(fullfile(trainDatFolder, '*.mat'));
    
    for i = 1:numel(fileList)
        load(strcat(trainDatFolder,fileList(i).name));
        
        % Load rotor inputs
        datName = erase(fileList(i).name, '.mat');
        trainInp(:,i) = dataStruct.(datName).measIn;
    
        % Build input matrix
        blk = [trainInp(1,i)*eye(6), trainInp(2,i)*eye(6),... 
               trainInp(3,i)*eye(6), trainInp(4,i)*eye(6),... 
               trainInp(5,i)*eye(6), trainInp(6,i)*eye(6),...
               trainInp(7,i)*eye(6), trainInp(8,i)*eye(6)];
        U = [U;blk];
    
        % Load output wrench 
        trainOutp(:,i) = dataStruct.(datName).measWrench;
        Z = [Z;trainOutp(:,i)];
    
        % Build weighted norm matrix
        w = diag([10;10;10;1e-12;1e-12;1e-12]);   
        W = blkdiag(W,w);
    end

    % Ask if the user wants to continue
    answer = questdlg('Do you want to select more data?', ...
                      'Continue?', ...
                      'Yes', 'No', 'No');
        
    if strcmp(answer, 'No')
        break;
    end    
end

%% Solve for allocation matrix
validDatFolder = '/home/rveenstra/MTP/data/region3-train4/data/';

% Range of deltas tested
del = [1e-6,5e-6,1e-5,3e-5,5e-5,1e-4];

% Plot mean error of validation data of optimized wrench map
figure(1)
hold on
grid on
xlabel('\Delta'); ylabel('Mean error');

for j=1:numel(del)
    H = (U' * W * U);
    H = (H + H') / 2;
    f = -(U' * W * Z);
    
    % Ensure wrench map is close to nominal 
    wrenchMapNom = reshape(config.uavParams.wrenchMap, [], 1);
    delta = del(j);
    lb = wrenchMapNom - delta;
    ub = wrenchMapNom + delta;
    
    % Solve QP problem
    [wMap, fval] = quadprog(H,f,[],[],[],[],lb,ub);
    
    % Extract wrench map matrix
    wrenchMap(:,:,j) = reshape(wMap,6,8);
    
    % Validation
    fileList = dir(fullfile(validDatFolder, '*.mat'));
    
    for i = 1:numel(fileList)
        load(strcat(validDatFolder,fileList(i).name));
    
        % Load measured rotor inputs
        datName = erase(fileList(i).name, '.mat');
        validInp(:,i) = dataStruct.(datName).measIn;
    
        % Predicted output
        expectedOutp(:,i) = wrenchMap(:,:,j)*validInp(:,i);
        expectedOutpNom(:,i) = config.uavParams.wrenchMap*validInp(:,i);
    
        % Load measured output wrench 
        validOutp(:,i) = dataStruct.(datName).measWrench;
    end
    
    % Compute mean errors
    errOpt = vecnorm(expectedOutp(1:3,:) - validOutp(1:3,:));
    meanErrOpt = mean(errOpt);

    errNom = vecnorm(expectedOutpNom(1:3,:) - validOutp(1:3,:));
    meanErrNom = mean(errNom);
    
    plot(del(j),meanErrOpt,"o","MarkerFaceColor","r")
end
title('Delta vs. mean error')

%% Plot measured and computed thrust vectors
visualDatFolder = '/home/rveenstra/MTP/data/region3-train4/data/';

fileList = dir(fullfile(visualDatFolder, '*.mat'));

% Check if all vectors are good fit
figure(2)
hold on
view(3)
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');

origin = [0,0,0];
for i=1:numel(fileList)
    load(strcat(visualDatFolder,fileList(i).name));
    datName = erase(fileList(i).name, '.mat');
    compWrench = wrenchMap(:,:,5)*dataStruct.(datName).measIn;
    compNomWrench = config.uavParams.wrenchMap*dataStruct.(datName).measIn;
    measWrench = dataStruct.(datName).measWrench;

    quiver3(origin(1), origin(2), origin(3), ...
            compWrench(1), compWrench(2), compWrench(3), ...
            0, 'LineWidth', 2, 'Color', 'r');
    drawnow
    pause(0.1)    
    quiver3(origin(1), origin(2), origin(3), ...
            compNomWrench(1), compNomWrench(2), compNomWrench(3), ...
            0, 'LineWidth', 2, 'Color', 'b');
    drawnow
    pause(0.1)
    quiver3(origin(1), origin(2), origin(3), ...
            measWrench(1),measWrench(2),measWrench(3),...
            0, 'LineWidth', 3, 'Color', 'g');
    drawnow
    pause(0.1)
end
legend('Optimization wrench map \Delta = 5e-5', 'Nominal modelled wrench map', 'FT sensor measurement');
title('Comparison measured and wrench map computed thrusts') 

%% Save wrench map(s)
saveFolder = '/home/rveenstra/MTP/git/ms2025-ruben_veenstra/matlab/omnimorph_sim/wMap/Flight/wrenchMap_reg3.mat';

wMapStruct.wrenchMap = wrenchMap;
wMapStruct.delta = del;
wMapStruct.W = W;
wMapStruct.U = U;
wMapStruct.Z = Z;

save(saveFolder, 'wMapStruct');
