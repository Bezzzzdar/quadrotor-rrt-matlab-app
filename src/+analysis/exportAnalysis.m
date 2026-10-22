function exportAnalysis(results, variants, resultDir)
%EXPORTANALYSIS Записывает по книге на метод и по озаглавленному листу на параметр.
% https://www.mathworks.com/help/matlab/ref/writetable.html
    if ~isfolder(resultDir), mkdir(resultDir); end
    methods = {'rrt', 'astar', 'voronoi'};
    for methodIndex = 1:numel(methods)
        method = methods{methodIndex};
        file = fullfile(resultDir, ['analysis_', method, '.xlsx']);
        names = fieldnames(results.(method).tables);
        for parameter = 1:numel(names)
            name = names{parameter};
            label = strrep(variants.(method).(name).label, '\lambda', char(955));
            heading = ['Параметр: ', label, ' (', name, ')'];
            mode = 'overwritesheet';
            if parameter == 1, mode = 'replacefile'; end
            writetable(analysis.excelSafeTable(results.(method).tables.(name)), file, ...
                'Sheet', name, 'Range', 'A3', 'WriteMode', mode, 'UseExcel', false);
            writecell({heading}, file, 'Sheet', name, 'Range', 'A1', ...
                'WriteMode', 'inplace', 'UseExcel', false);
        end
    end
end
