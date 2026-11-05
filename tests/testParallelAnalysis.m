function tests = testParallelAnalysis
%TESTPARALLELANALYSIS Проверяет процессы, независимость RNG и равенство геометрических результатов.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подготавливает общие небольшие карты и двухпроцессный пул.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.oldRng = rng;
    testCase.TestData.ownsPool = false;
    addpath(genpath(fullfile(root, 'src')));
    assumeTrue(testCase, exist('parpool', 'file') == 2 && license('test', 'Distrib_Computing_Toolbox'));
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    existing = gcp('nocreate');
    assumeFalse(testCase, isa(existing, 'parallel.ThreadPool'));
    testCase.TestData.ownsPool = isempty(existing);
    scenario = config.defaultScenario();
    scenario.map = struct('width', 40, 'height', 40, 'resolution', 1);
    scenario.start2d = [3, 3];
    scenario.goal2d = [37, 37];
    scenario.safetyRadius = 1;
    scenario.sim.vRef = 5;
    scenario.sim.dt = 0.1;
    configurations = analysis.baselineConfigurations();
    configurations(1).Parameters.stepSize = 5;
    configurations(1).Parameters.goalBias = 0.3;
    configurations(1).Parameters.maxIter = 1000;
    configurations(1).Parameters.smoothingIterations = 20;
    options = struct('sceneCount', 4, 'plannerSeedBase', 1001, 'sceneSeedBase', 6001, ...
        'useParallel', false, 'maxWorkers', 2, 'showProgress', false, ...
        'obstacles', struct('countRange', [1, 3], 'minSize_m', [3, 3], ...
        'maxSize_m', [7, 7], 'maxAreaFraction', 0.3, 'maxPlacementAttempts', 1000));
    scenes = analysis.generateScenes(scenario, options);
    serial = analysis.runExperiment(scenario, configurations, options, scenes);
    options.useParallel = true;
    options.showProgress = true;
    before = rng;
    parallel = analysis.runExperiment(scenario, configurations, options, scenes);
    testCase.TestData.rngPreserved = isequal(rng, before);
    testCase.TestData.scenario = scenario;
    testCase.TestData.configurations = configurations;
    testCase.TestData.options = options;
    testCase.TestData.scenes = scenes;
    testCase.TestData.serial = serial;
    testCase.TestData.parallel = parallel;
end

function teardownOnce(testCase)
%TEARDOWNONCE Закрывает только пул, созданный данным набором тестов.
    if testCase.TestData.ownsPool
        pool = gcp('nocreate');
        if ~isempty(pool), delete(pool); end
    end
    rng(testCase.TestData.oldRng);
    path(testCase.TestData.oldPath);
end

function testParallelWorkersAndRng(testCase)
%TESTPARALLELWORKERSANDRNG Подтверждает реальное выполнение на разных процессах.
    actual = testCase.TestData.parallel;
    verifyTrue(testCase, actual.execution.useParallel);
    verifyEqual(testCase, actual.execution.mode, 'parallel');
    verifyEqual(testCase, actual.execution.workerCount, 2);
    verifyGreaterThanOrEqual(testCase, numel(unique(actual.samples.WorkerId)), 2);
    verifyTrue(testCase, all(actual.samples.WorkerId > 0));
    verifyTrue(testCase, testCase.TestData.rngPreserved);
    verifyEqual(testCase, actual.execution.trialCount, 12);
    verifyGreaterThan(testCase, actual.execution.elapsed_s, 0);
    verifyFalse(testCase, any(actual.samples.Status == "error"));
end

function testSerialAndParallelAgree(testCase)
%TESTSERIALANDPARALLELAGREE Сравнивает всё, кроме времени и идентификатора процесса.
    serial = testCase.TestData.serial;
    parallel = testCase.TestData.parallel;
    verifyEqual(testCase, serial.scenes, parallel.scenes);
    verifyEqual(testCase, serial.probabilities, parallel.probabilities);
    fields = {'Configuration', 'Planner', 'ConfigurationIndex', 'SceneId', 'SceneSeed', ...
        'PlannerSeed', 'Solved', 'Status', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'};
    verifyEqual(testCase, parallel.samples(:, fields), serial.samples(:, fields), 'AbsTol', 1e-9);
    verifyEqual(testCase, parallel.samples.PlanningTime_s, ...
        parallel.samples.AlgorithmTime_s + parallel.samples.MinimumSnapTime_s, 'AbsTol', 1e-12);
end

function testParameterQueueAgrees(testCase)
%TESTPARAMETERQUEUEAGREES Проверяет общую очередь значений параметров без вложенного parfor.
    scenario = testCase.TestData.scenario;
    configurations = testCase.TestData.configurations;
    options = testCase.TestData.options;
    variants.rrt.stepSize = struct('value', [4, 8], 'label', 'Шаг');
    variants.astar.heuristicType = struct('value', ["euclidean", "octile"], 'label', 'Эвристика');
    variants.voronoi.connectionCount = struct('value', [1, 3], 'label', 'Подключения');
    options.showProgress = false;
    options.useParallel = false;
    serial = analysis.runParameterStudy(scenario, configurations, variants, options, testCase.TestData.scenes);
    options.useParallel = true;
    parallel = analysis.runParameterStudy(scenario, configurations, variants, options, testCase.TestData.scenes);
    verifyTrue(testCase, parallel.execution.useParallel);
    verifyEqual(testCase, parallel.execution.trialCount, 24);
    for configuration = configurations
        planner = char(configuration.Planner);
        parameters = fieldnames(variants.(planner));
        for index = 1:numel(parameters)
            name = parameters{index};
            expected = serial.methods.(planner).(name);
            actual = parallel.methods.(planner).(name);
            fields = setdiff(actual.table.Properties.VariableNames, {'MeanPlanningTime_s', 'VariancePlanningTime_s'});
            verifyEqual(testCase, actual.table(:, fields), expected.table(:, fields), 'AbsTol', 1e-9);
            verifyEqual(testCase, actual.configurations, expected.configurations);
            verifyEqual(testCase, actual.samples.SceneId, repmat((1:4)', 2, 1));
            verifyEqual(testCase, actual.samples.PlannerSeed, repmat((1001:1004)', 2, 1));
        end
    end
end
