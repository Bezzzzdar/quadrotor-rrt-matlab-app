function [figures, parameterStudy] = plotSavedParameterStudy(resultDir)
%PLOTSAVEDPARAMETERSTUDY Перестраивает окна из MAT без планирования и запуска пула.

    filename = fullfile(resultDir, 'parameter_study.mat');
    if ~isfile(filename)
        error('viz:MissingStudyResults', ...
            'Нет файла %s. Сначала выполните run_analysis.m в режиме experiment.', filename);
    end
    saved = load(filename, 'parameterStudy');
    if ~isfield(saved, 'parameterStudy') || ~isstruct(saved.parameterStudy) || ...
            ~isscalar(saved.parameterStudy) || ...
            ~all(isfield(saved.parameterStudy, {'configurations', 'methods', 'variants'}))
        error('viz:InvalidStudyResults', 'В файле %s отсутствует структура parameterStudy.', filename);
    end
    parameterStudy = saved.parameterStudy;
    outputDir = fullfile(resultDir, 'parameters');
    if ~isfolder(outputDir), mkdir(outputDir); end
    figures = gobjects(numel(parameterStudy.configurations), 1);
    for index = 1:numel(figures)
        planner = char(parameterStudy.configurations(index).Planner);
        figures(index) = viz.plotParameterStudy(parameterStudy, planner);
        savefig(figures(index), fullfile(outputDir, ['parameters_', planner, '.fig']));
    end
end
