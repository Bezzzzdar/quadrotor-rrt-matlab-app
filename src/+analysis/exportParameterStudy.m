function exportParameterStudy(study, resultDir)
%EXPORTPARAMETERSTUDY Сохраняет моменты, вероятности и все попытки по параметрам.
% Каждый метод получает книгу Excel с отдельным листом для каждого параметра.

    if ~isfolder(resultDir), mkdir(resultDir); end
    for methodIndex = 1:numel(study.configurations)
        configuration = study.configurations(methodIndex);
        planner = char(configuration.Planner);
        parameters = fieldnames(study.methods.(planner));
        workbook = fullfile(resultDir, ['parameters_', planner, '.xlsx']);
        for parameterIndex = 1:numel(parameters)
            parameter = parameters{parameterIndex};
            result = study.methods.(planner).(parameter);
            mode = 'overwritesheet';
            if parameterIndex == 1, mode = 'replacefile'; end
            writetable(analysis.excelSafeTable(result.table), workbook, ...
                'Sheet', parameter, 'Range', 'A3', 'WriteMode', mode, 'UseExcel', false);
            writecell({configuration.Name, study.variants.(planner).(parameter).label}, ...
                workbook, 'Sheet', parameter, 'Range', 'A1', ...
                'WriteMode', 'inplace', 'UseExcel', false);
            stem = fullfile(resultDir, [planner, '_', parameter]);
            writetable(result.table, [stem, '.csv']);
            writetable(result.samples, [stem, '_runs.csv']);
        end
    end
end
