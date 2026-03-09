function trajToLog(pdes, vdes, ades, R, om, tSamples, saveName)
% trajToLog - Generate and save a trajectory log file.
%
%   trajToLog(pdes, vdes, ades, R, om, tSamples, saveName)
%
% This function generates a trajectory log file containing position,
% velocity, acceleration, orientation (as Euler angles), and angular
% velocity data, sampled over time. The data is written to a text file
% with a predefined header format.
%
% Inputs:
%   pdes      - Desired positions [3 x N] (x, y, z) at each time sample
%   vdes      - Desired linear velocities [3 x N] (vx, vy, vz)
%   ades      - Desired linear accelerations [3 x N] (ax, ay, az)
%   R         - Rotation matrices [3 x 3 x N] defining orientation
%   om        - Angular velocities [3 x N] (wx, wy, wz)
%   tSamples  - Time samples [1 x N], relative time in seconds
%   saveName  - Path and filename for the trajectory log (string)
%
% Output:
%   A log file is generated at the specified location, containing:
%       - Timestamp (Unix time, seconds)
%       - Position:      x, y, z
%       - Orientation:   roll, pitch, yaw
%       - Linear vel.:   vx, vy, vz
%       - Angular vel.:  wx, wy, wz
%       - Linear acc.:   ax, ay, az
%       - Angular acc.:  dwx, dwy, dwz (zeros by default)
%       - Jerk & higher derivatives (zeros by default)

    % Convert Rotation matrices to Euler angles
    eul = [];
    for i=1:size(R,3)
        eul(:,i) = (rotm2eul(R(:,:,i)))';
    end
    
    % Generate timestamps
    start_datetime = datetime;
    start_unix_time = posixtime(start_datetime);
    unix_times = start_unix_time + tSamples;
    datalen = length(tSamples);
    
    % Create file
    header = ['ts x y z roll pitch yaw vx vy vz wx wy wz ax ay az dwx dwy dwz jx jy jz ddwx ddwy ddwz\n'];
    data = [unix_times', pdes(1,:)', pdes(2,:)', pdes(3,:)', eul(3,:)', eul(2,:)', eul(1,:)', vdes(1,:)', vdes(2,:)', vdes(3,:)', om(1,:)', om(2,:)', om(3,:)', ades(1,:)', ades(2,:)', ades(3,:)', zeros(1,datalen)', zeros(1,datalen)', zeros(1,datalen)',zeros(1,datalen)', zeros(1,datalen)', zeros(1,datalen)', zeros(1,datalen)', zeros(1,datalen)', zeros(1,datalen)'];
    
    if isfile(saveName)
        delete(saveName); % Delete if already present
    end
    
    fid = fopen(saveName, 'a');
    
    fprintf(fid, header);
    
    formatSpec = '%.9f'; % for timestamp
    for i = 1:24
        formatSpec = [formatSpec, ' %.8g'];
    end
    
    formatSpec = [formatSpec, '\n'];
    
    for i = 1:length(unix_times)
        fprintf(fid, formatSpec, data(i, :));
    end
    
    fclose(fid);
end