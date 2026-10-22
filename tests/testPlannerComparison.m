function tests = testPlannerComparison
%TESTPLANNERCOMPARISON Проверяет сравнение по статье и сохранение всех исходов.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Создаёт небольшую общую карту и сохраняет окружение.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.oldRng = rng;
    addpath(genpath(fullfile(root, 'src')));
    scenario = config.defaultScenario();
    scenario.map = struct('width', 40, 'height', 40, 'resolution', 1);
    scenario.start2d = [3, 3];
    scenario.goal2d = [37, 37];
    scenario.obstacles = [15, 8, 4, 22];
    scenario.safetyRadius = 1;
    scenario.rrt.stepSize = 5;
    scenario.rrt.goalBias = 0.3;
    scenario.rrt.maxIter = 1000;
    scenario.rrt.smoothingIterations = 20;
    scenario.sim.vRef = 5;
    scenario.sim.dt = 0.1;
    testCase.TestData.scenario = scenario;
    testCase.TestData.configurations = struct('Name', {'RRT', 'A*', 'Вороной'}, ...
        'Planner', {'rrt', 'astar', 'voronoi'}, ...
        'Parameters', {scenario.rrt, scenario.astar, scenario.voronoi});
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает путь и генератор случайных чисел.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.oldRng);
end

function testContinuousCollisionBetweenSamples(testCase)
%TESTCONTINUOUSCOLLISIONBETWEENSAMPLES Находит пересечение дуги при безопасных концах.
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    pp = mkpp([0, 1], [0, 8, 1; -4, 4, 1], 2);
    [free, detail] = geometry.isPolynomialTrajectoryFree(pp, map, [4.5, 1.8, 1, 0.4], 0);
    verifyFalse(testCase, free);
    verifyEqual(testCase, detail.obstacle, 1);
    verifyTrue(testCase, all(ppval(pp, [0, 1]) < [10; 1.8], 'all'));
    verifyTrue(testCase, geometry.isPolynomialTrajectoryFree(pp, map, [4.5, 2.01, 1, 1], 0));
    verifyFalse(testCase, geometry.isPolynomialTrajectoryFree(pp, map, [4.5, 2, 1, 1], 0));
    map.height = 1.5;
    verifyFalse(testCase, geometry.isPolynomialTrajectoryFree(pp, map, zeros(0, 4), 0));
end

function testPreparedPlanPreservesPipeline(testCase)
%TESTPREPAREDPLANPRESERVESPIPELINE Не планирует второй раз и сохраняет случайные сокращения.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = testCase.TestData.scenario;
    rng(1001, 'twister');
    expected = runPlannerExperiment(scenario, 'rrt', false);
    planned = analysis.planSeeded(scenario, 'rrt', 1001);
    rng(planned.rngState);
    actual = runPlannerExperiment(scenario, 'rrt', false, planned);
    verifyEqual(testCase, actual.rawPath, expected.rawPath);
    verifyEqual(testCase, actual.path, expected.path, 'AbsTol', 1e-12);
    verifyEqual(testCase, actual.calculationTime, planned.calculationTime);
    verifyEqual(testCase, actual.metrics, expected.metrics);
    verifyError(testCase, @() runPlannerExperiment(scenario, 'astar', false, planned), ...
        'planning:PlannerMismatch');
end

function testSequentialComparisonAndReproducibility(testCase)
%TESTSEQUENTIALCOMPARISONANDREPRODUCIBILITY Проверяет фиксированную карту, seed и порядок.
    assumeTrue(testCase, exist('parpool', 'file') == 2 && exist('minsnappolytraj', 'file') == 2);
    options = struct('runCount', 2, 'seedBase', 1001, 'timeLimit_s', 10);
    oldRng = rng;
    result = analysis.runComparison(testCase.TestData.scenario, testCase.TestData.configurations, options);
    verifyEqual(testCase, rng, oldRng);
    verifyEqual(testCase, result.scenario, testCase.TestData.scenario);
    verifyEqual(testCase, height(result.samples), 6);
    verifyEqual(testCase, height(result.statistics), 12);
    verifyTrue(testCase, all(result.samples.PlanningSolved));
    verifyFalse(testCase, any(result.samples.Status == "error" | result.samples.Status == "timeout"));
    verifyTrue(testCase, all(result.samples.StartedAt(2:end) >= result.samples.FinishedAt(1:end-1)));
    verifyEqual(testCase, result.samples.Configuration, ["RRT"; "A*"; "Вороной"; "A*"; "Вороной"; "RRT"]);
    verifyEqual(testCase, result.samples.Seed, [1001; 1001; 1001; 1002; 1002; 1002]);
    verifyNotEmpty(testCase, result.metadata.sourceSnapshot);
    verifyNotEmpty(testCase, result.metadata.toolboxes);
    verifyEqual(testCase, result.metadata.warmupStatus, repmat("solved", 3, 1));
end

function testNoPathAndErrorsAreNotZeros(testCase)
%TESTNOPATHANDERRORSARENOTZEROS Сохраняет разные причины отсутствия решения.
    assumeTrue(testCase, exist('parpool', 'file') == 2);
    configurations = testCase.TestData.configurations(1:2);
    configurations(1).Parameters.maxIter = 1;
    configurations(1).Parameters.stepSize = 1;
    configurations(2).Parameters.heuristicType = 'unknown';
    options = struct('runCount', 1, 'seedBase', 1001, 'timeLimit_s', 10);
    result = analysis.runComparison(testCase.TestData.scenario, configurations, options);
    verifyEqual(testCase, result.samples.Status, ["no_path"; "error"]);
    verifyTrue(testCase, all(isnan(result.samples.PathLength_m)));
    verifyTrue(testCase, all(result.statistics.AvailableRuns == 0));
    verifyEqual(testCase, result.summary.NoPathRuns, [1; 0]);
    verifyEqual(testCase, result.summary.ErrorRuns, [0; 1]);
    verifyNotEmpty(testCase, char(result.samples.ErrorIdentifier(2)));
end

function testTimeoutIsRecorded(testCase)
%TESTTIMEOUTISRECORDED Отменяет задачу и не подставляет лимит вместо неизвестного времени.
    assumeTrue(testCase, exist('parpool', 'file') == 2);
    options = struct('runCount', 1, 'seedBase', 1001, 'timeLimit_s', 1e-6);
    result = analysis.runComparison(testCase.TestData.scenario, testCase.TestData.configurations(2), options);
    verifyEqual(testCase, result.samples.Status, "timeout");
    verifyTrue(testCase, isnan(result.samples.CalculationTime_s));
    verifyEqual(testCase, result.summary.TimeoutRuns, 1);
    pool = gcp('nocreate');
    if ~isempty(pool)
        verifyEmpty(testCase, pool.FevalQueue.RunningFutures);
        verifyEmpty(testCase, pool.FevalQueue.QueuedFutures);
    end
end

function testInvalidTrajectoryIsNotExecutionError(testCase)
%TESTINVALIDTRAJECTORYISNOTEXECUTIONERROR Сохраняет отказ общей проверки отдельно от исключений.
    assumeTrue(testCase, exist('parpool', 'file') == 2 && exist('minsnappolytraj', 'file') == 2);
    options = struct('runCount', 1, 'seedBase', 47, 'timeLimit_s', 10);
    result = analysis.runComparison(testCase.TestData.scenario, testCase.TestData.configurations(1), options);
    verifyEqual(testCase, result.samples.Status, "invalid_trajectory");
    verifyTrue(testCase, result.samples.PlanningSolved);
    verifyFalse(testCase, result.samples.Solved);
    verifyTrue(testCase, isnan(result.samples.PathLength_m));
    verifyEqual(testCase, result.summary.InvalidTrajectoryRuns, 1);
    verifyEqual(testCase, result.summary.ErrorRuns, 0);
    verifyGreaterThan(testCase, result.samples.ValidationTime_s, 0);
    verifyEqual(testCase, result.samples.PipelineTime_s, result.samples.CalculationTime_s + ...
        result.samples.PostprocessingTime_s + result.samples.ValidationTime_s, 'AbsTol', 1e-12);
end

function result = syntheticResult
%SYNTHETICRESULT Задаёт две успешные попытки и одну ошибку с известной статистикой.
    samples = table(repmat("RRT", 3, 1), ["solved"; "solved"; "error"], ...
        [true; true; false], [true; true; false], [0.1; 0.2; NaN], ...
        [1; 2; NaN], [100; 110; NaN], [0.01; Inf; NaN], [0.001; 0.002; NaN], ...
        'VariableNames', {'Configuration', 'Status', 'PlanningSolved', 'Solved', ...
        'CalculationTime_s', 'PipelineTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'});
    configurations = struct('Name', 'RRT', 'Planner', 'rrt', 'Parameters', struct('stepSize', 5));
    [summary, statistics] = analysis.summarizeComparison(samples, configurations);
    result = struct('samples', samples, 'configurations', configurations, ...
        'summary', summary, 'statistics', statistics, ...
        'metadata', struct('options', struct('timeLimit_s', 30), 'toolboxes', ver));
end

function testStatisticsPreserveFailuresAndInfinity(testCase)
%TESTSTATISTICSPRESERVEFAILURESANDINFINITY Не скрывает пропуски и неограниченную кривизну.
    result = syntheticResult;
    verifyEqual(testCase, result.summary.TrajectorySuccessRate, 2/3);
    verifyEqual(testCase, result.statistics.MissingRuns, ones(4, 1));
    verifyEqual(testCase, result.statistics.Mean(1), 0.15, 'AbsTol', eps);
    verifyEqual(testCase, result.statistics.Mean(2), 105);
    verifyEqual(testCase, result.statistics.Mean(3), Inf);
    verifyEqual(testCase, result.statistics.UnboundedRuns(3), 1);
end

function testFiguresAndUnconditionalECDF(testCase)
%TESTFIGURESANDUNCONDITIONALECDF Проверяет четыре критерия и знаменатель из всех попыток.
    result = syntheticResult;
    criteria = viz.plotPlannerComparison(result);
    reliability = viz.plotComparisonReliability(result);
    cleanup = onCleanup(@() close([criteria, reliability]));
    verifyNumElements(testCase, findobj(criteria, 'Type', 'axes'), 4);
    verifyNumElements(testCase, findobj(reliability, 'Type', 'axes'), 2);
    for metric = 1:4
        axis = findobj(criteria, 'Tag', sprintf('ComparisonMetric%d', metric));
        verifyEqual(testCase, axis.XTickLabel, {'RRT (n=2/3)'});
    end
    axis = findobj(criteria, 'Tag', 'ComparisonMetric1');
    verifyEqual(testCase, axis.YScale, 'log');
    for panel = 1:2
        curve = findobj(reliability, 'Tag', sprintf('ComparisonCDF%d_1', panel));
        verifyEqual(testCase, curve.YData(end), 2/3);
    end
    verifyNotEmpty(testCase, findobj(criteria, 'String', 'Inf: 1'));
    axesHandles = findall([criteria, reliability], 'Type', 'axes');
    for index = 1:numel(axesHandles)
        verifyEqual(testCase, axesHandles(index).FontName, 'Times');
        verifyEqual(testCase, axesHandles(index).FontSize, 14);
        verifyEqual(testCase, axesHandles(index).XColor, [0, 0, 0]);
        verifyEqual(testCase, axesHandles(index).YColor, [0, 0, 0]);
    end
end

function testExportIncludesAllRunsAndInfinityText(testCase)
%TESTEXPORTINCLUDESALLRUNSANDINFINITYTEXT Проверяет все листы и текстовый Inf в Excel.
    result = syntheticResult;
    folder = tempname;
    mkdir(folder);
    cleanup = onCleanup(@() removeTemporaryFolder(folder));
    analysis.exportComparison(result, folder);
    analysis.exportComparison(result, folder);
    workbook = fullfile(folder, 'planner_comparison.xlsx');
    verifyEqual(testCase, sheetnames(workbook), ["Summary"; "Statistics"; "Runs"; "Configurations"; "Metadata"]);
    rows = readtable(workbook, 'Sheet', 'Runs', 'Range', 'A3');
    verifyEqual(testCase, height(rows), 3);
    data = readcell(workbook, 'Sheet', 'Runs', 'Range', 'H5');
    verifyEqual(testCase, string(data{1}), "Inf");
    saved = load(fullfile(folder, 'planner_comparison.mat'));
    verifyEqual(testCase, saved.result.samples.MaxCurvature_1_m(2), Inf);
end

function removeTemporaryFolder(folder)
%REMOVETEMPORARYFOLDER Удаляет только созданную тестом папку внутри системного temp.
    if isfolder(folder) && startsWith([folder, filesep], tempdir, 'IgnoreCase', true)
        rmdir(folder, 's');
    end
end
