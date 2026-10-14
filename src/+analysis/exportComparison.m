function exportComparison(result, resultDir)
%EXPORTCOMPARISON Сохраняет все попытки, статистику, настройки и метаданные.
% В отличие от OMPL используются MAT, CSV и XLSX, не совместимый с OMPL SQL-лог.
% https://www.kavrakilab.org/publications/moll-sucan2015benchmarking-motion-planning.pdf
    if ~isfolder(resultDir), mkdir(resultDir); end
    count = numel(result.configurations);
    configurationNames = strings(count, 1);
    methods = configurationNames;
    parameters = configurationNames;
    for index = 1:count
        configurationNames(index) = string(result.configurations(index).Name);
        methods(index) = string(result.configurations(index).Planner);
        parameters(index) = string(jsonencode(result.configurations(index).Parameters));
    end
    configurations = table(configurationNames, methods, parameters, ...
        'VariableNames', {'Configuration', 'Planner', 'Parameters_json'});
    metadata = result.metadata;
    metadata = rmfield(metadata, 'toolboxes');
    if isfield(metadata, 'sourceSnapshot'), metadata = rmfield(metadata, 'sourceSnapshot'); end
    keys = fieldnames(metadata);
    values = strings(numel(keys), 1);
    for index = 1:numel(keys)
        value = metadata.(keys{index});
        if isdatetime(value)
            values(index) = string(value);
        else
            values(index) = string(jsonencode(value));
        end
    end
    metadataTable = table(string(keys), values, 'VariableNames', {'Key', 'Value'});
    sheets = {'Summary', 'Statistics', 'Runs', 'Configurations', 'Metadata'};
    titles = {'Исходы всех запусков', 'Статистика четырёх критериев', ...
        'Все отдельные попытки', 'Фиксированные настройки методов', 'Метаданные эксперимента'};
    tables = {result.summary, result.statistics, result.samples, configurations, metadataTable};
    workbook = fullfile(resultDir, 'planner_comparison.xlsx');
    for index = 1:numel(sheets)
        mode = 'overwritesheet';
        if index == 1, mode = 'replacefile'; end
        excelTable = tables{index};
        for name = excelTable.Properties.VariableNames
            value = excelTable.(name{1});
            if isnumeric(value) && any(isinf(value), 'all')
                % Excel не хранит числовой Inf: пишем текст, не допускаем подмены конечным числом.
                cells = num2cell(value);
                cells(value == Inf) = {'Inf'};
                cells(value == -Inf) = {'-Inf'};
                excelTable.(name{1}) = cells;
            end
        end
        writetable(excelTable, workbook, 'Sheet', sheets{index}, 'Range', 'A3', ...
            'WriteMode', mode, 'UseExcel', false);
        writecell(titles(index), workbook, 'Sheet', sheets{index}, 'Range', 'A1', ...
            'WriteMode', 'inplace', 'UseExcel', false);
    end
    writetable(result.samples, fullfile(resultDir, 'planner_comparison_runs.csv'));
    writetable(result.statistics, fullfile(resultDir, 'planner_comparison_statistics.csv'));
    save(fullfile(resultDir, 'planner_comparison.mat'), 'result');
end
