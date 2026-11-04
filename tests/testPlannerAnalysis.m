function tests = testPlannerAnalysis
%TESTPLANNERANALYSIS Проверяет таблицы методов и отдельные вероятности без графиков.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает исходники и задаёт небольшую карту для проверок.
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
    configurations = analysis.baselineConfigurations();
    configurations(1).Parameters = scenario.rrt;
    testCase.TestData.scenario = scenario;
    testCase.TestData.configurations = configurations;
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает окружение MATLAB.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.oldRng);
end

function testFourRowsAndSeparateProbability(testCase)
%TESTFOURROWSANDSEPARATEPROBABILITY Исключает неудачу из всех четырёх критериев.
    [samples, configurations] = syntheticSamples();
    [tables, probabilities] = analysis.summarizeResults(samples, configurations);
    verifySize(testCase, tables.rrt, [4, 3]);
    verifyEqual(testCase, tables.rrt.Properties.VariableNames, {'Criterion', 'Mean', 'Variance'});
    verifyEqual(testCase, tables.rrt.Mean, [2; 12; 0.2; 0.02], 'AbsTol', 1e-14);
    verifyEqual(testCase, tables.rrt.Variance, [2; 8; 0.02; 0.0002], 'AbsTol', 1e-14);
    verifyEqual(testCase, probabilities.rrt, 2/3);
end

function testMissingAndSingleObservation(testCase)
%TESTMISSINGANDSINGLEOBSERVATION Не заменяет отсутствие данных нулевой дисперсией.
    [samples, configurations] = syntheticSamples();
    samples.Solved(:) = false;
    [tables, probabilities] = analysis.summarizeResults(samples, configurations);
    verifyTrue(testCase, all(isnan(tables.rrt.Mean)));
    verifyTrue(testCase, all(isnan(tables.rrt.Variance)));
    verifyEqual(testCase, probabilities.rrt, 0);
    samples.Solved(1) = true;
    [tables, probabilities] = analysis.summarizeResults(samples, configurations);
    verifyEqual(testCase, tables.rrt.Mean, [1; 10; 0.1; 0.01], 'AbsTol', 1e-14);
    verifyTrue(testCase, all(isnan(tables.rrt.Variance)));
    verifyEqual(testCase, probabilities.rrt, 1/3);
end

function testInfinityIsNotDropped(testCase)
%TESTINFINITYISNOTDROPPED Сохраняет неограниченную кривизну допустимой траектории.
    [samples, configurations] = syntheticSamples();
    samples.MaxCurvature_1_m(2) = Inf;
    [tables, probabilities] = analysis.summarizeResults(samples, configurations);
    verifyEqual(testCase, tables.rrt.Mean(3), Inf);
    verifyTrue(testCase, isnan(tables.rrt.Variance(3)));
    verifyEqual(testCase, probabilities.rrt, 2/3);
end

function testCommonScenesAndFixedParameters(testCase)
%TESTCOMMONSCENESANDFIXEDPARAMETERS Проверяет общие карты, seed, отсутствие окон и сумму времён.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    options = smallOptions(2);
    oldRng = rng;
    figures = findall(groot, 'Type', 'figure');
    result = analysis.runExperiment(testCase.TestData.scenario, testCase.TestData.configurations, options);
    verifyEqual(testCase, rng, oldRng);
    verifyEqual(testCase, findall(groot, 'Type', 'figure'), figures);
    verifyEqual(testCase, result.configurations, testCase.TestData.configurations);
    verifyEqual(testCase, result.scenes, analysis.generateScenes(testCase.TestData.scenario, options));
    verifyNotEqual(testCase, result.scenes(1).obstacles, result.scenes(2).obstacles);
    verifyEqual(testCase, fieldnames(result.tables), {'rrt'; 'astar'; 'voronoi'});
    for method = {'rrt', 'astar', 'voronoi'}
        verifySize(testCase, result.tables.(method{1}), [4, 3]);
        verifyEqual(testCase, result.probabilities.(method{1}), 1);
    end
    verifyEqual(testCase, height(result.samples), 6);
    verifyTrue(testCase, all(result.samples.Solved));
    verifyEqual(testCase, result.samples.SceneId, [1; 1; 1; 2; 2; 2]);
    verifyEqual(testCase, result.samples.SceneSeed, [5001; 5001; 5001; 5002; 5002; 5002]);
    verifyEqual(testCase, result.samples.PlannerSeed, [1001; 1001; 1001; 1002; 1002; 1002]);
    verifyEqual(testCase, result.samples.PlanningTime_s, ...
        result.samples.AlgorithmTime_s + result.samples.MinimumSnapTime_s, 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, result.samples.MinimumSnapTime_s, zeros(6, 1));
end

function testUnsafeMinimumSnapIsNotSolved(testCase)
%TESTUNSAFEMINIMUMSNAPISNOTSOLVED Не принимает свободную ломаную за допустимую траекторию.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    options = smallOptions(1);
    options.plannerSeedBase = 47;
    options.sceneSeedBase = 5018;
    result = analysis.runExperiment(testCase.TestData.scenario, ...
        testCase.TestData.configurations(1), options);
    verifyEqual(testCase, result.samples.Status, "invalid_trajectory");
    verifyFalse(testCase, result.samples.Solved);
    verifyTrue(testCase, isnan(result.samples.PathLength_m));
    verifyTrue(testCase, all(isnan(result.tables.rrt.Mean)));
    verifyEqual(testCase, result.probabilities.rrt, 0);
    verifyNotEmpty(testCase, char(result.samples.Message));
end

function testFailuresRemainInProbabilityDenominator(testCase)
%TESTFAILURESREMAININPROBABILITYDENOMINATOR Учитывает отсутствие пути и исключения.
    configurations = testCase.TestData.configurations(1:2);
    configurations(1).Parameters.maxIter = 1;
    configurations(1).Parameters.stepSize = 1;
    configurations(2).Parameters.heuristicType = 'unknown';
    result = analysis.runExperiment(testCase.TestData.scenario, configurations, smallOptions(1));
    verifyEqual(testCase, result.samples.Status, ["no_path"; "error"]);
    verifyEqual(testCase, [result.probabilities.rrt, result.probabilities.astar], [0, 0]);
    verifyTrue(testCase, all(isnan(result.tables.rrt{:, 2:end}), 'all'));
    verifyTrue(testCase, all(isnan(result.tables.astar{:, 2:end}), 'all'));
    verifyTrue(testCase, contains(result.samples.Message(2), 'astar:InvalidHeuristic'));
end

function testMinimumSnapTimeIsMeasuredSeparately(testCase)
%TESTMINIMUMSNAPTIMEISMEASUREDSEPARATELY Проверяет отдельный замер построения и сумму времён.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = testCase.TestData.scenario;
    scenario.obstacles = zeros(0, 4);
    planned = struct('plannerName', 'astar', 'displayName', 'A*', ...
        'path', [scenario.start2d; scenario.goal2d], 'tree', [], 'calculationTime', 0.125);
    started = tic;
    result = runPlannerExperiment(scenario, 'astar', false, planned);
    elapsed = toc(started);
    verifyTrue(testCase, result.pathFound);
    verifyEmpty(testCase, result.simLog);
    verifyEqual(testCase, result.calculationTime, 0.125);
    verifyGreaterThan(testCase, result.minimumSnapTime, 0);
    verifyLessThanOrEqual(testCase, result.minimumSnapTime, elapsed);
    verifyEqual(testCase, result.planningTime, 0.125 + result.minimumSnapTime, 'AbsTol', 1e-12);
end

function testSceneGenerationIsReproducible(testCase)
%TESTSCENEGENERATIONISREPRODUCIBLE Проверяет сетку, геометрию и неизменность общего ГСЧ.
    scenario = testCase.TestData.scenario;
    options = smallOptions(10);
    oldRng = rng;
    scenes = analysis.generateScenes(scenario, options);
    verifyEqual(testCase, rng, oldRng);
    verifyEqual(testCase, scenes, analysis.generateScenes(scenario, options));
    for index = 1:numel(scenes)
        obstacles = scenes(index).obstacles;
        geometry.validateScenario(scenario.map, scenario.start2d, scenario.goal2d, ...
            obstacles, scenario.safetyRadius);
        verifySize(testCase, obstacles, [3, 4]);
        verifyEqual(testCase, mod(obstacles, scenario.map.resolution), zeros(3, 4));
        verifyLessThanOrEqual(testCase, scenes(index).areaFraction, 0.3);
    end
    options.sceneSeedBase = 6001;
    changed = analysis.generateScenes(scenario, options);
    verifyNotEqual(testCase, scenes(1).obstacles, changed(1).obstacles);
end

function testDisconnectedMapsAreNotRejected(testCase)
%TESTDISCONNECTEDMAPSARENOTREJECTED Сохраняет в выборке карту с непроходимой стеной.
    scenario = testCase.TestData.scenario;
    options = smallOptions(1);
    options.obstacles.countRange = [1, 1];
    options.obstacles.minSize_m = [4, 40];
    options.obstacles.maxSize_m = [4, 40];
    scenes = analysis.generateScenes(scenario, options);
    scenario.obstacles = scenes.obstacles;
    planned = planGeometricPath(scenario, 'astar');
    verifyEmpty(testCase, planned.path);
    verifyEqual(testCase, scenes.areaFraction, 0.1);
end

function testRandomCountsAndDimensionsRespectConstraints(testCase)
%TESTRANDOMCOUNTSANDDIMENSIONSRESPECTCONSTRAINTS Проверяет случайные числа, размеры и ограничения.
    scenario = testCase.TestData.scenario;
    options = smallOptions(40);
    options.obstacles.countRange = [1, 12];
    options.obstacles.maxSize_m = [12, 12];
    options.obstacles.maxPlacementAttempts = 10000;
    oldRng = rng;
    scenes = analysis.generateScenes(scenario, options);
    verifyEqual(testCase, rng, oldRng);
    verifyEqual(testCase, scenes, analysis.generateScenes(scenario, options));
    counts = arrayfun(@(scene) size(scene.obstacles, 1), scenes);
    verifyGreaterThan(testCase, numel(unique(counts)), 1);
    verifyGreaterThanOrEqual(testCase, counts, ones(size(counts)));
    verifyLessThanOrEqual(testCase, counts, 12*ones(size(counts)));
    allObstacles = vertcat(scenes.obstacles);
    verifyGreaterThan(testCase, numel(unique(allObstacles(:, 3))), 1);
    verifyGreaterThan(testCase, numel(unique(allObstacles(:, 4))), 1);
    for index = 1:numel(scenes)
        obstacles = scenes(index).obstacles;
        dimensions = obstacles(:, 3:4);
        verifyGreaterThanOrEqual(testCase, dimensions, ...
            repmat(options.obstacles.minSize_m, size(obstacles, 1), 1));
        verifyLessThanOrEqual(testCase, dimensions, ...
            repmat(options.obstacles.maxSize_m, size(obstacles, 1), 1));
        verifyEqual(testCase, mod(obstacles, scenario.map.resolution), zeros(size(obstacles)));
        areaFraction = sum(prod(dimensions, 2)) / (scenario.map.width*scenario.map.height);
        verifyEqual(testCase, scenes(index).areaFraction, areaFraction, 'AbsTol', 1e-14);
        verifyLessThanOrEqual(testCase, areaFraction, options.obstacles.maxAreaFraction);
        verifyFalse(testCase, geometry.isPointInObstacles( ...
            scenario.start2d, obstacles, scenario.safetyRadius));
        verifyFalse(testCase, geometry.isPointInObstacles( ...
            scenario.goal2d, obstacles, scenario.safetyRadius));
        geometry.validateScenario(scenario.map, scenario.start2d, scenario.goal2d, ...
            obstacles, scenario.safetyRadius);
        for obstacleIndex = 1:size(obstacles, 1)
            previous = obstacles(1:obstacleIndex-1, :);
            candidate = obstacles(obstacleIndex, :);
            intersects = all(candidate(1:2) < previous(:, 1:2) + previous(:, 3:4) & ...
                candidate(1:2) + candidate(3:4) > previous(:, 1:2), 2);
            verifyFalse(testCase, any(intersects));
        end
    end
end

function testEmptySceneIsAllowed(testCase)
%TESTEMPTYSCENEISALLOWED Допускает явно заданный нулевой диапазон количества препятствий.
    options = smallOptions(1);
    options.obstacles.countRange = [0, 0];
    scenes = analysis.generateScenes(testCase.TestData.scenario, options);
    verifySize(testCase, scenes.obstacles, [0, 4]);
    verifyEqual(testCase, scenes.areaFraction, 0);
    verifyEqual(testCase, scenes.placementAttempts, 0);
end

function testInvalidCountRangeIsRejected(testCase)
%TESTINVALIDCOUNTRANGEISREJECTED Отклоняет обратный диапазон и заведомое превышение площади.
    scenario = testCase.TestData.scenario;
    options = smallOptions(1);
    options.obstacles.countRange = [5, 2];
    verifyError(testCase, @() analysis.generateScenes(scenario, options), 'analysis:InvalidObstacleSettings');
    options.obstacles.countRange = [1, 54];
    verifyError(testCase, @() analysis.generateScenes(scenario, options), 'analysis:InvalidObstacleSettings');
end

function testImpossibleGenerationFailsExplicitly(testCase)
%TESTIMPOSSIBLEGENERATIONFAILSEXPLICITLY Не подменяет неразмещённую карту исходными данными.
    scenario = testCase.TestData.scenario;
    options = smallOptions(1);
    options.obstacles.minSize_m = [25, 25];
    options.obstacles.maxSize_m = [25, 25];
    verifyError(testCase, @() analysis.generateScenes(scenario, options), 'analysis:InvalidObstacleSettings');
    options = smallOptions(1);
    options.obstacles.maxPlacementAttempts = 5;
    scenario.safetyRadius = 100;
    verifyError(testCase, @() analysis.generateScenes(scenario, options), 'analysis:SceneGenerationFailed');
end

function [samples, configurations] = syntheticSamples()
%SYNTHETICSAMPLES Задаёт две допустимые траектории и одну неудачу с известными моментами.
    configurations = struct('Name', 'RRT', 'Planner', 'rrt', 'Parameters', struct());
    samples = table(repmat("RRT", 3, 1), [true; true; false], [1; 3; 1000], ...
        [10; 14; 1000], [0.1; 0.3; 1000], [0.01; 0.03; 1000], ...
        'VariableNames', {'Configuration', 'Solved', 'PlanningTime_s', ...
        'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'});
end

function options = smallOptions(sceneCount)
%SMALLOPTIONS Задаёт небольшие случайные карты для быстрых проверок.
    options = struct('sceneCount', sceneCount, 'plannerSeedBase', 1001, 'sceneSeedBase', 5001, ...
        'obstacles', struct('countRange', [3, 3], 'minSize_m', [3, 3], 'maxSize_m', [7, 7], ...
        'maxAreaFraction', 0.3, 'maxPlacementAttempts', 1000));
end
