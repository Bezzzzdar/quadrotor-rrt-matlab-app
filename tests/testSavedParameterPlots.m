function tests = testSavedParameterPlots
%TESTSAVEDPARAMETERPLOTS Проверяет перестроение из MAT без эксперимента и маркеров.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает исходники и сохраняет настройки графики.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.oldVisible = get(groot, 'DefaultFigureVisible');
    addpath(genpath(fullfile(root, 'src')));
    set(groot, 'DefaultFigureVisible', 'off');
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает настройки MATLAB.
    set(groot, 'DefaultFigureVisible', testCase.TestData.oldVisible);
    path(testCase.TestData.oldPath);
end

function testMissingCacheDoesNotRunExperiment(testCase)
%TESTMISSINGCACHEDOESNOTRUNEXPERIMENT Возвращает ошибку без неявного запуска расчёта.
    verifyError(testCase, @() viz.plotSavedParameterStudy(tempname), 'viz:MissingStudyResults');
end

function testInvalidCache(testCase)
%TESTINVALIDCACHE Отклоняет MAT с несовместимой структурой.
    folder = tempname;
    mkdir(folder);
    cleanup = onCleanup(@() rmdir(folder, 's'));
    parameterStudy = struct('invalid', true);
    save(fullfile(folder, 'parameter_study.mat'), 'parameterStudy');
    verifyError(testCase, @() viz.plotSavedParameterStudy(folder), 'viz:InvalidStudyResults');
end

function testRedrawFromSavedData(testCase)
%TESTREDRAWFROMSAVEDDATA Подтверждает отсутствие вызовов планировщиков и изменения MAT.
    folder = tempname;
    mkdir(folder);
    folderCleanup = onCleanup(@() rmdir(folder, 's'));
    parameterStudy = syntheticStudy();
    filename = fullfile(folder, 'parameter_study.mat');
    save(filename, 'parameterStudy');
    before = dir(filename);
    oldRng = rng;
    pool = [];
    hasParallel = exist('gcp', 'file') == 2 && license('test', 'Distrib_Computing_Toolbox');
    if hasParallel, pool = gcp('nocreate'); end
    [figures, loaded] = viz.plotSavedParameterStudy(folder);
    figureCleanup = onCleanup(@() close(figures));
    after = dir(filename);
    verifyEqual(testCase, loaded, parameterStudy);
    verifyEqual(testCase, rng, oldRng);
    verifyEqual(testCase, [after.bytes, after.datenum], [before.bytes, before.datenum]);
    if hasParallel, verifyEqual(testCase, gcp('nocreate'), pool); end
    verifyNumElements(testCase, figures, 3);
    for index = 1:numel(figures)
        verifyNumElements(testCase, findall(figures(index), 'Type', 'uitab'), 5);
        series = findall(figures(index), '-regexp', 'Tag', '^StudySeries_');
        verifyNumElements(testCase, series, 27);
        for item = series'
            if strcmp(item.Type, 'line')
                verifyEqual(testCase, item.LineStyle, '-');
                verifyEqual(testCase, item.Marker, 'none');
            else
                verifyEqual(testCase, item.Type, 'bar');
            end
            verifyTrue(testCase, all(isfinite(item.YData)));
        end
        if strcmp(parameterStudy.configurations(index).Planner, 'astar')
            verifyNumElements(testCase, findall(figures(index), 'Type', 'bar'), 18);
        end
        planner = char(parameterStudy.configurations(index).Planner);
        verifyTrue(testCase, isfile(fullfile(folder, 'parameters', ['parameters_', planner, '.fig'])));
    end
end

function study = syntheticStudy()
%SYNTHETICSTUDY Задаёт только готовые оценки: исходных данных для планирования в MAT нет.
    study.configurations = analysis.baselineConfigurations();
    for configuration = study.configurations
        planner = char(configuration.Planner);
        names = fieldnames(configuration.Parameters);
        if strcmp(planner, 'rrt'), names = names(1:3); end
        for index = 1:numel(names)
            name = names{index};
            values = [1; 2];
            if strcmp(name, 'heuristicType'), values = ["euclidean"; "octile"]; end
            if strcmp(name, 'tieBreaking'), values = ["largerg"; "smallerg"]; end
            summary = table(values, 'VariableNames', {'ParameterValue'});
            summary.Probability = [0.5; 0.75];
            for metric = {'PlanningTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'}
                summary.(['Mean', metric{1}]) = [1; 2];
                summary.(['Variance', metric{1}]) = [0.1; 0.2];
            end
            study.methods.(planner).(name).table = summary;
            study.variants.(planner).(name) = struct('value', values, 'label', name);
        end
    end
end
