function checkResult(response, commandDescription, exception)
% Extract genom3 result and give warning if command failed
    if ~(strcmp(response, 'done') || strcmp(response, 'sent')) 
        warning('Command "%s" failed.', commandDescription);
        
        fprintf('Exception: \n');
        printStruct(exception)
    end
end