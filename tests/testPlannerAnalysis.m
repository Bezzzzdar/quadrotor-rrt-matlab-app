function tests = testPlannerAnalysis
%TESTPLANNERANALYSIS Проверяет повторные замеры, параллельность и шкалы графиков.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает исходники и создаёт небольшой сценарий для проверок.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.rng = rng;
    addpath(genpath(fullfile(root, 'src')));
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = config.defaultScenario();
    scenario.map = struct('width', 40, 'height', 40, 'resolution', 1);
    scenario.start2d = [3, 3];
    scenario.goal2d = [37, 37];
    scenario.obstacles = [15, 8, 4, 22];
    scenario.safetyRadius = 1;
    scenario.rrt.stepSize = 5;
    scenario.rrt.goalBias = 0.3;
    scenario.rrt.maxIter = 500;
    scenario.rrt.smoothingIterations = 20;
    scenario.sim.vRef = 5;
    scenario.sim.dt = 0.1;
    testCase.TestData.scenario = scenario;
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает окружение MATLAB.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.rng);
end

function testTenRepeatsForEachCategory(testCase)
%TESTTENREPEATSFOREACHCATEGORY Сохраняет десять отдельных замеров каждого типа.
    variants.heuristicType = struct('value', ["euclidean", "octile"], 'label', 'Эвристика');
    actual = analysis.runMethod(testCase.TestData.scenario, 'astar', variants, 10, 42);
    samples = actual.samples.heuristicType;
    verifyEqual(testCase, height(samples), 20);
    verifyEqual(testCase, sum(samples.ParameterValue == "euclidean"), 10);
    verifyEqual(testCase, sum(samples.ParameterValue == "octile"), 10);
    verifyEqual(testCase, actual.tables.heuristicType.MeanPathLength_m(1), ...
        mean(samples.PathLength_m(samples.ParameterValue == "euclidean")));
    verifyEqual(testCase, actual.tables.heuristicType.ErrorCount, zeros(2, 1));
end

function testFailuresAreRecorded(testCase)
%TESTFAILURESARERECORDED Не скрывает исключения пустым catch или ложным успехом.
    variants.heuristicType = struct('value', ["euclidean", "unknown"], 'label', 'Эвристика');
    actual = analysis.runMethod(testCase.TestData.scenario, 'astar', variants, 2, 42);
    verifyEqual(testCase, actual.tables.heuristicType.ErrorCount, [0; 2]);
    verifyEqual(testCase, actual.tables.heuristicType.PathFoundRate, [1; 0]);
    verifyTrue(testCase, contains(actual.tables.heuristicType.Message(2), 'astar:InvalidHeuristic'));
    verifyTrue(testCase, isnan(actual.tables.heuristicType.MeanPathLength_m(2)));
end

function testThreeParallelMethods(testCase)
%TESTTHREEPARALLELMETHODS Проверяет три задачи и пересечение интервалов их выполнения.
    assumeTrue(testCase, exist('parpool', 'file') == 2);
    variants.rrt.stepSize = struct('value', [5, 8], 'label', 'Шаг');
    variants.rrt.goalBias = struct('value', [0.2, 0.4], 'label', 'Вероятность');
    variants.rrt.maxIter = struct('value', [300, 500], 'label', 'Итерации');
    variants.astar.heuristicType = struct('value', ["euclidean", "octile"], 'label', 'Эвристика');
    variants.astar.heuristicWeight = struct('value', [0.5, 1], 'label', 'Вес');
    variants.astar.tieBreaking = struct('value', ["largerg", "smallerg"], 'label', 'Равенства');
    variants.voronoi.clearanceWeight = struct('value', [9, 30], 'label', 'Вес');
    variants.voronoi.clearanceExponent = struct('value', [1, 1.5], 'label', 'Степень');
    variants.voronoi.connectionCount = struct('value', [3, 5], 'label', 'Подключения');
    actual = analysis.runParallel(testCase.TestData.scenario, variants, 2, 42);
    methods = {'rrt', 'astar', 'voronoi'};
    workers = zeros(1, 3);
    starts = NaT(1, 3, 'TimeZone', 'UTC');
    ends = starts;
    for index = 1:3
        value = actual.(methods{index});
        workers(index) = value.workerTaskID;
        starts(index) = value.startedAt;
        ends(index) = value.finishedAt;
        names = fieldnames(variants.(methods{index}));
        for parameter = 1:3
            verifyEqual(testCase, height(value.tables.(names{parameter})), 2);
            verifyEqual(testCase, height(value.samples.(names{parameter})), 4);
            verifyEqual(testCase, value.tables.(names{parameter}).ErrorCount, zeros(2, 1));
        end
    end
    verifyEqual(testCase, numel(unique(workers)), 3);
    verifyLessThan(testCase, max(starts), min(ends));
end

function testFourPlotsAndThreeParameterAxes(testCase)
%TESTFOURPLOTSANDTHREEPARAMETERAXES Проверяет числовые, логарифмические и текстовые шкалы.
    variants.a = struct('value', [1, 100], 'label', 'Вес', 'scale', 'log');
    variants.b = struct('value', [0.5, 2], 'label', 'Степень');
    variants.c = struct('value', ["largerg", "smallerg"], 'label', 'Правило', ...
        'tickLabels', {{'Больший g', 'Меньший g'}});
    names = fieldnames(variants);
    result.tables = struct();
    for index = 1:3
        result.tables.(names{index}) = table(variants.(names{index}).value(:), [0.1; 0.2], ...
            [50; 60], [0.01; 0.02], [0.001; 0.002], ...
            'VariableNames', {'ParameterValue', 'MeanCalculationTime_s', 'MeanPathLength_m', ...
            'MeanMaxCurvature_1_m', 'MeanMeanCurvature_1_m'});
    end
    result.tables.a.MeanMaxCurvature_1_m(2) = Inf;
    fig = viz.plotPlannerAnalysis(result, variants, 'Проверка');
    cleanup = onCleanup(@() close(fig));
    for metric = 1:4
        main = findobj(fig, 'Tag', sprintf('AnalysisMetric%d', metric));
        verifyNumElements(testCase, findobj(main, 'Type', 'line'), 3);
        previous = Inf;
        for parameter = 1:3
            scale = findobj(fig, 'Tag', sprintf('AnalysisParameter%d_%d', metric, parameter));
            verifyLessThan(testCase, scale.Position(2), previous);
            previous = scale.Position(2);
        end
    end
    scale = findobj(fig, 'Tag', 'AnalysisParameter1_1');
    verifyEqual(testCase, scale.XScale, 'log');
    scale = findobj(fig, 'Tag', 'AnalysisParameter1_3');
    verifyEqual(testCase, scale.XTickLabel, {'Больший g'; 'Меньший g'});
    main = findobj(fig, 'Tag', 'AnalysisMetric3');
    verifyNotEmpty(testCase, findobj(main, 'String', 'Inf'));
    axesHandles = findobj(fig, 'Type', 'axes');
    for index = 1:numel(axesHandles)
        axis = axesHandles(index);
        verifyEqual(testCase, axis.XColor, [0, 0, 0]);
        verifyEqual(testCase, axis.YColor, [0, 0, 0]);
        verifyEqual(testCase, axis.ZColor, [0, 0, 0]);
        verifyEqual(testCase, axis.FontName, 'Times');
        verifyEqual(testCase, axis.FontSize, 14);
        verifyEqual(testCase, axis.FontUnits, 'points');
    end
    labels = findall(fig, 'Type', 'text');
    for index = 1:numel(labels)
        if isempty(labels(index).String), continue; end
        verifyEqual(testCase, labels(index).FontName, 'Times');
        verifyEqual(testCase, labels(index).FontSize, 14);
        verifyEqual(testCase, labels(index).Color, [0, 0, 0]);
    end
    legends = findobj(fig, 'Type', 'legend');
    for index = 1:numel(legends)
        verifyEqual(testCase, legends(index).FontName, 'Times');
        verifyEqual(testCase, legends(index).FontSize, 14);
        verifyEqual(testCase, legends(index).TextColor, [0, 0, 0]);
    end
end
