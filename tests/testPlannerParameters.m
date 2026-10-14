function tests = testPlannerParameters
%TESTPLANNERPARAMETERS Проверяет настройки планировщиков и неизменность критериев.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает исходники и сохраняет окружение MATLAB.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.rng = rng;
    addpath(genpath(fullfile(root, 'src')));
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает окружение MATLAB.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.rng);
end

function testAStarOptionsPreserveOptimalCost(testCase)
%TESTASTAROPTIONSPRESERVEOPTIMALCOST Сравнивает допустимые эвристики с поиском почти без h.
% Условия оптимальности: https://www.cs.cmu.edu/~motionplanning/lecture/Asearch_v8.pdf
    map = struct('width', 30, 'height', 30, 'resolution', 1);
    obstacles = [10, 5, 5, 17];
    reference = astar.plan([2, 2], [28, 28], map, obstacles, 1, struct('heuristicWeight', eps));
    cost = metrics.calculatePathLength(reference);
    for heuristic = {'euclidean', 'octile'}
        for tie = {'largerg', 'smallerg'}
            for weight = [0.25, 1]
                options = struct('heuristicType', heuristic{1}, 'heuristicWeight', weight, 'tieBreaking', tie{1});
                [route, search] = astar.plan([2, 2], [28, 28], map, obstacles, 1, options);
                verifyEqual(testCase, metrics.calculatePathLength(route), cost, 'AbsTol', 1e-10);
                verifyEqual(testCase, search.heuristicType, string(heuristic{1}));
                verifyEqual(testCase, search.tieBreaking, string(tie{1}));
                verifyEqual(testCase, search.heuristicWeight, weight);
            end
        end
    end
end

function testOctileHandlesShortBoundarySteps(testCase)
%TESTOCTILEHANDLESSHORTBOUNDARYSTEPS Проверяет согласованную оценку неравномерного края.
    map = struct('width', 5.6, 'height', 4.3, 'resolution', 1);
    reference = astar.plan([0, 0], [5.6, 4.3], map, zeros(0, 4), 0, struct('heuristicWeight', eps));
    actual = astar.plan([0, 0], [5.6, 4.3], map, zeros(0, 4), 0, struct('heuristicType', 'octile'));
    verifyEqual(testCase, metrics.calculatePathLength(actual), ...
        metrics.calculatePathLength(reference), 'AbsTol', 1e-12);
end

function testInvalidAStarSettings(testCase)
%TESTINVALIDASTARSETTINGS Не допускает неизвестные правила и неоптимальные веса.
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    verifyError(testCase, @() astar.plan([0, 0], [10, 10], map, zeros(0, 4), 0, ...
        struct('heuristicType', 'unknown')), 'astar:InvalidHeuristic');
    verifyError(testCase, @() astar.plan([0, 0], [10, 10], map, zeros(0, 4), 0, ...
        struct('tieBreaking', 'unknown')), 'astar:InvalidTieBreaking');
end

function testVoronoiPenaltyExponent(testCase)
%TESTVORONOIPENALTYEXPONENT Проверяет расширенный штраф и прежний вариант p=1.
% Исходная стоимость: Малые БПЛА. Теория и практика (2015), формула 12.1.
    nodes = [0, 0; 10, 0];
    edges = [1, 2];
    sites = [5, 2];
    verifyEqual(testCase, voronoi.edgeCosts(nodes, edges, sites, 0.1, 0.9, 2), ...
        0.1*10 + 0.9/4, 'AbsTol', 1e-12);
    verifyEqual(testCase, voronoi.edgeCosts(nodes, edges, sites, 0.1, 0.9, 1), ...
        voronoi.edgeCosts(nodes, edges, sites, 0.1, 0.9));
end

function testVoronoiConnectionsAndWeight(testCase)
%TESTVORONOICONNECTIONSANDWEIGHT Изменяет K и стоимость без изменения карты.
    map = struct('width', 20, 'height', 20, 'resolution', 1);
    for count = [1, 3, 5]
        params = struct('clearanceWeight', 30, 'clearanceExponent', 1.5, 'connectionCount', count);
        [route, roadmap] = voronoi.plan([3.2, 4.1], [16.3, 17.1], map, zeros(0, 4), 0, params);
        verifyNotEmpty(testCase, route);
        verifyEqual(testCase, size(roadmap.startConnection, 1), count);
        verifyEqual(testCase, size(roadmap.goalConnection, 1), count);
        verifyEqual(testCase, roadmap.clearanceWeight, 30);
        verifyEqual(testCase, roadmap.weights, roadmap.k1*roadmap.edgeLengths + ...
            roadmap.k2./roadmap.edgeClearances.^1.5, 'AbsTol', 1e-12);
    end
end

function testAnalysisWithoutDynamicsPreservesCriteria(testCase)
%TESTANALYSISWITHOUTDYNAMICSPRESERVESCRITERIA Не меняет траекторию при пропуске слежения.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = config.defaultScenario();
    scenario.start2d = [10, 10];
    scenario.goal2d = [30, 30];
    scenario.obstacles = zeros(0, 4);
    scenario.rrt.goalBias = 1;
    scenario.rrt.maxIter = 2;
    rng(53);
    full = runPlannerExperiment(scenario, 'rrt');
    rng(53);
    referenceOnly = runPlannerExperiment(scenario, 'rrt', false);
    verifyEqual(testCase, referenceOnly.referenceTrajectory, full.referenceTrajectory);
    verifyEqual(testCase, referenceOnly.metrics, full.metrics);
    verifyEmpty(testCase, referenceOnly.simLog);
    verifyTrue(testCase, isnan(referenceOnly.summary.DynamicFeasible));
end
