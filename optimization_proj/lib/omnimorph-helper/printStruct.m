function printStruct(s, indent)
    if nargin < 2
        indent = '';
    end

    fields = fieldnames(s);
    for i = 1:numel(fields)
        field = fields{i};
        value = s.(field);
        
        if isstruct(value)
            fprintf('%s%s:\n', indent, field);
            if numel(value) == 1
                printStruct(value, [indent '  ']);  % Recursive call
            else
                for j = 1:numel(value)
                    fprintf('%s  Element %d:\n', indent, j);
                    printStruct(value(j), [indent '    ']);
                end
            end
        else
            fprintf('%s%s: ', indent, field);
            disp(value);
        end
    end
end