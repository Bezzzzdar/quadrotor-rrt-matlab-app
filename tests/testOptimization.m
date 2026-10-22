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

function testExactCollisionMatchesPolynomialCheck(testCase)
%TESTEXACTCOLLISIONMATCHESPOLYNOMIALCHECK Сопоставляет две независимые проверки отрезка.
    rng(27);
    map = struct('width', 100, 'height', 90);
    obstacles = [20, 30, 15, 20; 70, 5, 5, 20];
    points = [-1, 0; 0, 0; 100, 90; 20, 30; 70, 0; 70, 0; rand(100, 2)*110];
    for i = 1:2:size(points, 1)
        a = points(i, :);
        b = points(i+1, :);
        pp = mkpp([0, 1], [b(1)-a(1), a(1); b(2)-a(2), a(2)], 2);
        expected = geometry.isPolynomialTrajectoryFree(pp, map, obstacles, 2);
        verifyEqual(testCase, geometry.isSegmentCollisionFree(a, b, map, obstacles, 2), expected);
    end
end

function testPolylineLength(testCase)
%TESTPOLYLINELENGTH Проверяет длину исходного маршрута без уплотнения и дублирования точек.
    points = [0, 0; 3, 4; 3, 4; 3, 9];
    verifyEqual(testCase, metrics.calculatePathLength(points), 10);
    verifyEqual(testCase, metrics.calculatePathLength(points(1, :)), 0);
    verifyEqual(testCase, metrics.calculatePathLength([]), 0);
    verifyEqual(testCase, metrics.calculatePathLength([0, 0, 0; 0, 3, 4]), 5);
end

function testEmptyTrajectoryMetrics(testCase)
%TESTEMPTYTRAJECTORYMETRICS Отсутствие траектории не подменяется нулевыми критериями.
    actual = metrics.pathMetrics([]);
    verifyTrue(testCase, isnan(actual.pathLength));
    verifyTrue(testCase, isnan(actual.maxCurvature));
    verifyTrue(testCase, isnan(actual.meanCurvature));
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
    points = astar.plan([0.49, 0.49], [2, 0], map, ...
        [0.65, 0.1, 0.1, 0.35], 0, struct());
    verifyNotEmpty(testCase, points);
    verifyEqual(testCase, points(1, :), [0.49, 0.49]);
    verifyTrue(testCase, geometry.isPathCollisionFree(points, map, [0.65, 0.1, 0.1, 0.35], 0));
end

function testAStarShortBoundaryStepAndSameCell(testCase)
%TESTASTARSHORTBOUNDARYSTEPANDSAMECELL Сохраняет оба конца внутри одной клетки.
    map = struct('width', 2.6, 'height', 2.6, 'resolution', 1);
    points = astar.plan([2.55, 2.55], [2.6, 2.6], map, zeros(0, 4), 0, struct());
    verifyEqual(testCase, points, [2.55, 2.55; 2.6, 2.6]);
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
        points = actual.processedPath;
        verifyEqual(testCase, mod(size(points, 1), 2), 1);
        verifyEqual(testCase, points(2:2:end, :), ...
            points(1:2:end-2, :) + 0.5*(points(3:2:end, :) - points(1:2:end-2, :)));
        withoutShortcuts = scenario;
        withoutShortcuts.rrt.smoothingIterations = 0;
        rng(99);
        expected = runPlannerExperiment(withoutShortcuts, planner{1});
        verifyEqual(testCase, actual.rawPath, expected.rawPath);
        verifyEqual(testCase, actual.processedPath, expected.processedPath);
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
        scenario.safetyRadius, params.smoothingIterations);
    expectedRandomState = rng;
    rng(121314);
    actual = runPlannerExperiment(scenario, 'rrt');
    verifyEqual(testCase, actual.rawPath, rawPath);
    verifyEqual(testCase, actual.processedPath, expected);
    verifyEqual(testCase, rng, expectedRandomState);
end

function testSharedCurvaturePreparation(testCase)
%TESTSHAREDCURVATUREPREPARATION Общий расчёт эквивалентен отдельным функциям.
% Формулы: https://arxiv.org/pdf/math/0606007
% Поиск экстремумов: https://escholarship.org/uc/item/1p74v16x
    trajectory = mkpp([0, 1], [0, 1, 0; 1, 0, 0], 2);
    actual = metrics.pathMetrics(trajectory);
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
