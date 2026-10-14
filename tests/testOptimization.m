function tests = testOptimization
%TESTOPTIMIZATION Проверяет эквивалентность оптимизированных вычислений.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает исходники и сохраняет окружение теста.
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

function testSampledCollisionEquivalence(testCase)
%TESTSAMPLEDCOLLISIONEQUIVALENCE Сравнивает пакетную проверку с прежним циклом.
    rng(27);
    map = struct('width', 100, 'height', 90);
    obstacles = [20, 30, 15, 20; 70, 5, 5, 20];
    points = [-1, 0; 0, 0; 100, 90; 20, 30; 70, 0; 70, 0; rand(100, 2)*110];
    for i = 1:2:size(points, 1)
        a = points(i, :);
        b = points(i+1, :);
        for step = [0.5, 3, 200]
            n = max(2, ceil(norm(b-a)/step));
            expected = true;
            for k = 0:n
                point = a + (k/n)*(b-a);
                if ~geometry.isPointInsideMap(point, map) || ...
                        geometry.isPointInObstacles(point, obstacles, 2)
                    expected = false;
                    break;
                end
            end
            verifyEqual(testCase, geometry.isSegmentCollisionFree(a, b, map, obstacles, 2, step), expected);
        end
    end
end

function testInterpolationEquivalence(testCase)
%TESTINTERPOLATIONEQUIVALENCE Сохраняет точки, порядок и повторяющиеся вершины.
    points = [0, 0; 3, 4; 3, 4; 3, 9];
    expected = [0, 0; 1.5, 2; 3, 4; 3, 4; 3, 4; 3, 6.5; 3, 9];
    verifyEqual(testCase, metrics.interpolatePath(points, 3), expected);
    verifyEqual(testCase, metrics.interpolatePath(points(1, :), 3), points(1, :));
    verifyEmpty(testCase, metrics.interpolatePath([], 3));
end

function testDiscreteCurvature(testCase)
%TESTDISCRETECURVATURE Проверяет окружность, прямую и совпадающие точки.
    verifyEqual(testCase, metrics.calculateCurvature([1, 0; 0, 1; -1, 0]), 1, 'AbsTol', 1e-14);
    verifyEqual(testCase, metrics.calculateCurvature([0, 0; 1, 0; 2, 0]), 0);
    verifyEqual(testCase, metrics.calculateCurvature([0, 0; 0, 0; 2, 0]), []);
end

function testMotorMixerEquivalence(testCase)
%TESTMOTORMIXEREQUIVALENCE Сравнивает явные формулы с исходной матрицей.
    rng(81);
    quad = config.defaultQuadrotor();
    matrix = [1, 1, 1, 1; 0, -quad.d, 0, quad.d; ...
        quad.d, 0, -quad.d, 0; -quad.cTauF, quad.cTauF, -quad.cTauF, quad.cTauF];
    for i = 1:50
        command = randn(4, 1);
        expected = matrix \ command;
        motors = quadrotor.thrustMomentToMotors(command(1), command(2:4), quad);
        verifyEqual(testCase, motors, expected, 'AbsTol', 1e-12);
        [force, moment] = quadrotor.motorsToThrustMoment(motors', quad);
        verifyEqual(testCase, [force; moment], matrix*motors, 'AbsTol', 1e-12);
    end
end

function testAStarShortestOpenRoute(testCase)
%TESTASTARSHORTESTOPENROUTE Проверяет диагональную стоимость и концы пути.
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    [points, search] = astar.plan([0, 0], [10, 10], map, zeros(0, 4), 0, struct());
    verifyEqual(testCase, points, [(0:10)', (0:10)']);
    verifyEqual(testCase, metrics.calculatePathLength(points), 10*sqrt(2), 'AbsTol', 1e-12);
    verifyNotEmpty(testCase, search.visitedIds);
end

function testAStarBlockedAndOffGridEndpoints(testCase)
%TESTASTARBLOCKEDANDOFFGRIDENDPOINTS Не подменяет недопустимые концы маршрута.
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    [points, search] = astar.plan([0, 0], [10, 10], map, [5, 0, 1, 10], 0, struct());
    verifyEmpty(testCase, points);
    verifyNotEmpty(testCase, search.message);
    verifyEmpty(testCase, astar.plan([-1, 0], [10, 10], map, zeros(0, 4), 0, struct()));
    params = struct('collisionCheckStep', 0.01);
    [points, search] = astar.plan([0.49, 0.49], [2, 0], map, ...
        [0.65, 0.1, 0.1, 0.35], 0, params);
    verifyEmpty(testCase, points);
    verifyNotEmpty(testCase, search.message);
end

function testAStarShortBoundaryStepAndSameCell(testCase)
%TESTASTARSHORTBOUNDARYSTEPANDSAMECELL Сохраняет оба конца внутри одной клетки.
    map = struct('width', 2.6, 'height', 2.6, 'resolution', 1);
    points = astar.plan([2.55, 2.55], [2.6, 2.6], map, zeros(0, 4), 0, struct());
    verifyEqual(testCase, points, [2.55, 2.55; 2.6, 2.6]);
end

function testSingleExperimentCompatibility(testCase)
%TESTSINGLEEXPERIMENTCOMPATIBILITY Сохраняет формат старого запуска эксперимента.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = config.defaultScenario();
    scenario.start2d = [10, 10];
    scenario.goal2d = [30, 30];
    scenario.obstacles = zeros(0, 4);
    scenario.rrt.goalBias = 1;
    scenario.rrt.maxIter = 2;
    rng(37);
    expected = runPlannerExperiment(scenario, 'rrt');
    rng(37);
    actual = runSingleExperiment(scenario);
    verifyEqual(testCase, fieldnames(actual), fieldnames(rmfield(expected, {'plannerName', 'displayName'})));
    verifyEqual(testCase, actual.path, expected.path);
    verifyEqual(testCase, actual.metrics, expected.metrics);
    verifyEqual(testCase, actual.simLog, expected.simLog);
end

function testNonRRTDoesNotUseRandomShortcuts(testCase)
%TESTNONRRTDOESNOTUSERANDOMSHORTCUTS Исключает случайность, сохраняя середины сегментов.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = config.defaultScenario();
    for planner = {'astar', 'voronoi'}
        rng(53);
        initialRandomState = rng;
        actual = runPlannerExperiment(scenario, planner{1});
        verifyEqual(testCase, rng, initialRandomState);
        assertTrue(testCase, actual.pathFound);
        points = actual.rrtPath;
        verifyEqual(testCase, mod(size(points, 1), 2), 1);
        verifyEqual(testCase, points(2:2:end, :), ...
            points(1:2:end-2, :) + 0.5*(points(3:2:end, :) - points(1:2:end-2, :)));
        withoutShortcuts = scenario;
        withoutShortcuts.rrt.smoothingIterations = 0;
        rng(99);
        expected = runPlannerExperiment(withoutShortcuts, planner{1});
        verifyEqual(testCase, actual.rawPath, expected.rawPath);
        verifyEqual(testCase, actual.rrtPath, expected.rrtPath);
        verifyEqual(testCase, actual.referenceTrajectory.pp, expected.referenceTrajectory.pp);
        verifyEqual(testCase, actual.metrics, expected.metrics);
    end
end

function testRRTRetainsRandomShortcuts(testCase)
%TESTRRTRETAINSRANDOMSHORTCUTS Сохраняет маршрут и состояние генератора прежнего RRT.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = config.defaultScenario();
    params = scenario.rrt;
    rng(121314);
    rawPath = rrt.plan(scenario.start2d, scenario.goal2d, scenario.map, ...
        scenario.obstacles, scenario.safetyRadius, params);
    expected = rrt.shortcutSmoothPath(rawPath, scenario.map, scenario.obstacles, ...
        scenario.safetyRadius, params.collisionCheckStep, params.smoothingIterations);
    expectedRandomState = rng;
    rng(121314);
    actual = runPlannerExperiment(scenario, 'rrt');
    verifyEqual(testCase, actual.rawPath, rawPath);
    verifyEqual(testCase, actual.rrtPath, expected);
    verifyEqual(testCase, rng, expectedRandomState);
end

function testSharedCurvaturePreparation(testCase)
%TESTSHAREDCURVATUREPREPARATION Общий расчёт эквивалентен отдельным функциям.
% Формулы: https://arxiv.org/pdf/math/0606007
% Поиск экстремумов: https://escholarship.org/uc/item/1p74v16x
    trajectory = mkpp([0, 1], [0, 1, 0; 1, 0, 0], 2);
    points = ppval(trajectory, linspace(0, 1, 11))';
    actual = metrics.pathMetrics(points, 0.1, trajectory);
    [meanValue, meanInfo] = metrics.calculateMeanCurvature(trajectory);
    [maxValue, maxInfo] = metrics.calculateMaxCurvature(trajectory);
    verifyEqual(testCase, actual.meanCurvature, meanValue);
    verifyEqual(testCase, actual.maxCurvature, maxValue);
    verifyEqual(testCase, actual.curvatureMeanInfo, meanInfo);
    verifyEqual(testCase, actual.curvatureMaxInfo, maxInfo);
end

function testBatchedTreePlot(testCase)
%TESTBATCHEDTREEPLOT Рёбра дерева создают один графический объект.
    fig = figure('Visible', 'off');
    cleanup = onCleanup(@() close(fig));
    ax = axes(fig);
    tree = struct('nodes', [0, 0; 1, 0; 1, 1; 2, 1], 'parents', [0; 1; 2; 2]);
    viz.plotTree(ax, tree);
    lines = findobj(ax, 'Type', 'line');
    verifyNumElements(testCase, lines, 1);
    verifyEqual(testCase, lines.XData, [0, 1, NaN, 1, 1, NaN, 1, 2, NaN]);
    verifyEqual(testCase, lines.YData, [0, 0, NaN, 0, 1, NaN, 0, 1, NaN]);
end
