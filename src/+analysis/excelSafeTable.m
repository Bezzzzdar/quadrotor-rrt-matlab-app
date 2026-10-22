function result = excelSafeTable(input)
%EXCELSAFETABLE Сохраняет знак бесконечностей текстом вместо числовой подмены Excel.
% https://www.mathworks.com/help/matlab/ref/writetable.html
    result = input;
    for name = result.Properties.VariableNames
        value = result.(name{1});
        if isnumeric(value) && any(isinf(value), 'all')
            cells = num2cell(value);
            cells(value == Inf) = {'Inf'};
            cells(value == -Inf) = {'-Inf'};
            result.(name{1}) = cells;
        end
    end
end
