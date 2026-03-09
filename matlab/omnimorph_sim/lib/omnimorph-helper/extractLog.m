function extractLog(filename, outputVarName)
% extractLog - Loads and processes genom3 log data
%
%   extractLog(filename, outputVarName)
%
% Inputs:
%   filename   - Path to the log file
%   structName - Name of the struct (string)
%
    % Open the file
    fid = fopen(filename, 'r');
    if fid == -1
        error('Could not open file: %s', filename);
    end

    % Skip initial comment lines
    while true
        pos = ftell(fid);
        line = fgetl(fid);
        if ~ischar(line)
            error('No valid header found in file: %s', filename);
        end
        if ~startsWith(strtrim(line), '#')
            break;
        end
    end

    % Go back to header
    fseek(fid, pos, 'bof');

    % Read header line
    headerLine = fgetl(fid);
    headers = strsplit(strtrim(headerLine));
    numCols = numel(headers);

    % Build dynamic format string
    if strcmp(headers{1}, 'ts')
        formatSpec = repmat('%f', 1, numCols);
    else
        formatSpec = ['%s' repmat('%f', 1, numCols-1)];
    end

    % Read the data with '-' as empty (converted to NaN)
    dataArray = textscan(fid, formatSpec, 'TreatAsEmpty', '-');

    % Close the file
    fclose(fid);

    % Build struct
    logData = struct();
    for i = 1:numCols
        logData.(headers{i}) = dataArray{i};
    end

    % Assign to caller workspace
    assignin('caller', outputVarName, logData);
end