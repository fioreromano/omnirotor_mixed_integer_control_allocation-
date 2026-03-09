function extractFTSensorData(filename, outputVarName)
% extractFTSensorData - Loads and processes force-torque sensor data
%
%   dataStruct = loadFTSensorData(filename, structName)
%
% Inputs:
%   filename   - Path to the log file
%   structName - Name of the struct (string)
%
% Output:
%   dataStruct - Struct containing processed FT sensor data

    % Initialize struct
    FTSensorData = struct();

    % Open file 
    fid = fopen(filename, 'r');
    if fid == -1
        error('Could not open file: %s', filename);
    end
    
    % Skip initial lines
    for j=1:7
        fgetl(fid);
    end
    header = fgetl(fid);
        
    % Save force-torque data
    j = 1;
    while ~feof(fid)
        dat = fgetl(fid);
        datCell = split(dat, '"');
        FTSensorData.RDT(j) = str2double(datCell(4));
        FTSensorData.FT(j) = str2double(datCell(6));
        FTSensorData.Fx(j) = str2double(datCell(8));  
        FTSensorData.Fy(j) = str2double(datCell(10));
        FTSensorData.Fz(j) = str2double(datCell(12));
        FTSensorData.Tx(j) = str2double(datCell(14));
        FTSensorData.Ty(j) = str2double(datCell(16));
        FTSensorData.Tz(j) = str2double(datCell(18));
        FTSensorData.date(j) = datetime(datCell(20), 'InputFormat', 'MMM dd, yyyy, h:mm:ss a','TimeZone','Europe/Amsterdam');
            
        j = j + 1;
    end
        
    fclose(fid);
    
    % Process FT sensor data
    % Find initial second increment
    dates = FTSensorData.date;
    deltaT = seconds(diff(dates));
    ind = find(deltaT >= 1, 1, 'first');
    
    % Trim data
    FTSensorData.Fx = FTSensorData.Fx(ind+1:end);
    FTSensorData.Fy = FTSensorData.Fy(ind+1:end);
    FTSensorData.Fz = FTSensorData.Fz(ind+1:end);
    FTSensorData.Tx = FTSensorData.Tx(ind+1:end);
    FTSensorData.Ty = FTSensorData.Ty(ind+1:end);
    FTSensorData.Tz = FTSensorData.Tz(ind+1:end);

    % Create more accurate time field
    FTSensorData.date = datetime(FTSensorData.date(ind+1:end),'Format','DDD:HH:mm:ss.SSS');
    FTSensorData.RDT = FTSensorData.RDT(ind+1:end);
    FTSensorData.ts = FTSensorData.date(1) + milliseconds(0:numel(FTSensorData.date)-1); % RDT Sample rate = 1000

    % Assign to caller workspace
    assignin('caller', outputVarName, FTSensorData);
end