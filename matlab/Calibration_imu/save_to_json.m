% Example MATLAB variable
load('calib_29_11_24_std_fixed.mat');
calibration = eval('calibration');

% Convert the MATLAB variable to a JSON string
jsonStr = jsonencode(calibration);

% Specify the file name
filename = 'calib_29_11_24_std_fixed.json';

% Open the file for writing
fid = fopen(filename, 'w');

% Write the JSON string to the file
if fid == -1
    error('Cannot open file for writing: %s', filename);
else
    fprintf(fid, '%s', jsonStr);
    fclose(fid);
end

disp('JSON file saved successfully.');

