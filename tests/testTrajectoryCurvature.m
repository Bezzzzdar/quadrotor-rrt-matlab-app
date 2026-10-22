function tests = testTrajectoryCurvature
%TESTTRAJECTORYCURVATURE Проверяет геометрические метрики на известных кривых.
% Литература: https://arxiv.org/pdf/math/0606007
% https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает исходные функции проекта.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    addpath(genpath(fullfile(root, 'src')));
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает путь MATLAB.
    path(testCase.TestData.oldPath);
end

function testStraightLine(testCase)
%TESTSTRAIGHTLINE Кривизна прямой равна нулю.
    pp = mkpp([0, 2], [3, 1; 4, 2], 2);
    [meanValue, info] = metrics.calculateMeanCurvature(pp);
    verifyEqual(testCase, meanValue, 0);
    verifyEqual(testCase, metrics.calculateMaxCurvature(pp), 0);
    verifyEqual(testCase, info.pathLength, 10, 'AbsTol', 1e-10);
end

function testParabolaAndInteriorMaximum(testCase)
%TESTPARABOLAANDINTERIORMAXIMUM Для r=(u,(u-a)^2) максимум равен 2 при u=a.
    a = 0.37123;
    pp = mkpp([0, 1], [0, 1, 0; 1, -2*a, a^2], 2);
    [maximum, info] = metrics.calculateMaxCurvature(pp);
    [meanValue, meanInfo] = metrics.calculateMeanCurvature(pp);
    lengthValue = integral(@(u) sqrt(1 + 4*(u-a).^2), 0, 1);
    turning = atan(2*(1-a)) + atan(2*a);
    verifyEqual(testCase, maximum, 2, 'AbsTol', 1e-9);
    verifyEqual(testCase, info.time, a, 'AbsTol', 1e-8);
    verifyEqual(testCase, meanValue, turning/lengthValue, 'AbsTol', 1e-8);
    verifyEqual(testCase, meanInfo.totalCurvature, turning, 'AbsTol', 1e-8);
end

function testTimeReparameterizationAndRemovableStop(testCase)
%TESTTIMEREPARAMETERIZATIONANDREMOVABLESTOP Кривые (u,u^2) и (u^3,u^6) совпадают.
    regular = mkpp([0, 1], [0, 1, 0; 1, 0, 0], 2);
    stopped = mkpp([0, 10], [0, 0, 0, 1e-3, 0, 0, 0; 1e-6, 0, 0, 0, 0, 0, 0], 2);
    verifyEqual(testCase, metrics.calculateMeanCurvature(stopped), ...
        metrics.calculateMeanCurvature(regular), 'AbsTol', 1e-8);
    verifyEqual(testCase, metrics.calculateMaxCurvature(stopped), 2, 'AbsTol', 1e-8);
end

function testUnboundedEndpointWithFiniteMean(testCase)
%TESTUNBOUNDEDENDPOINTWITHFINITEMEAN Для (u^5,u^6) кривизна не ограничена у нуля.
    pp = mkpp([0, 1], [0, 1, 0, 0, 0, 0, 0; 1, 0, 0, 0, 0, 0, 0], 2);
    [maximum, info] = metrics.calculateMaxCurvature(pp);
    [meanValue, meanInfo] = metrics.calculateMeanCurvature(pp);
    lengthValue = integral(@(u) u.^4 .* sqrt(25 + 36*u.^2), 0, 1);
    verifyEqual(testCase, maximum, Inf);
    verifyTrue(testCase, info.unbounded);
    verifyEqual(testCase, info.time, 0);
    verifyEqual(testCase, meanInfo.totalCurvature, atan(6/5), 'AbsTol', 1e-8);
    verifyEqual(testCase, meanValue, atan(6/5)/lengthValue, 'AbsTol', 1e-8);
end

function testSpatialCurve(testCase)
%TESTSPATIALCURVE Проверяет неплоскую кривую (u,u^2,u^3).
    pp = mkpp([0, 1], [0, 0, 1, 0; 0, 1, 0, 0; 1, 0, 0, 0], 3);
    curvature = @(u) sqrt(36*u.^4 + 36*u.^2 + 4) ./ (1 + 4*u.^2 + 9*u.^4).^(3/2);
    speed = @(u) sqrt(1 + 4*u.^2 + 9*u.^4);
    expected = integral(@(u) curvature(u).*speed(u), 0, 1)/integral(speed, 0, 1);
    verifyEqual(testCase, metrics.calculateMeanCurvature(pp), expected, 'AbsTol', 1e-8);
    verifyEqual(testCase, metrics.calculateMaxCurvature(pp), 2, 'AbsTol', 1e-8);
end

function testPieceBoundaryAndCorner(testCase)
%TESTPIECEBOUNDARYANDCORNER Прямой угол добавляет pi/2 к полному повороту.
    pp = mkpp([0, 1, 2], [1, 0; 0, 0; 0, 1; 1, 0], 2);
    [meanValue, info] = metrics.calculateMeanCurvature(pp);
    verifyEqual(testCase, meanValue, pi/4, 'AbsTol', 1e-10);
    verifyEqual(testCase, info.cornerAngle, pi/2, 'AbsTol', 1e-10);
    verifyEqual(testCase, metrics.calculateMaxCurvature(pp), Inf);
end

function testInteriorReversal(testCase)
%TESTINTERIORREVERSAL Смена направления на прямой учитывается как поворот на pi.
    pp = mkpp([0, 1], [1, -1, 0.25; 0, 0, 0], 2);
    [meanValue, info] = metrics.calculateMeanCurvature(pp);
    verifyEqual(testCase, info.totalCurvature, pi, 'AbsTol', 1e-10);
    verifyEqual(testCase, meanValue, 2*pi, 'AbsTol', 1e-9);
    verifyEqual(testCase, metrics.calculateMaxCurvature(pp), Inf);
end

function testStationaryTrajectory(testCase)
%TESTSTATIONARYTRAJECTORY Для нулевой длины геометрические метрики не определены.
    pp = mkpp([0, 1], [3; 4], 2);
    verifyTrue(testCase, isnan(metrics.calculateMeanCurvature(pp)));
    verifyTrue(testCase, isnan(metrics.calculateMaxCurvature(pp)));
end

function testDiscontinuousPositionRejected(testCase)
%TESTDISCONTINUOUSPOSITIONREJECTED Разрыв координат нельзя считать непрерывным путём.
    pp = mkpp([0, 1, 2], [1, 0; 0, 0; 1, 10; 0, 0], 2);
    verifyError(testCase, @() metrics.calculateMeanCurvature(pp), 'metrics:DiscontinuousTrajectory');
end

function testMetricsIndependentOfSamples(testCase)
%TESTMETRICSINDEPENDENTOFSAMPLES Отсчёты отрисовки не влияют на полиномиальные метрики.
    pp = mkpp([0, 1], [0, 1, 0; 1, 0, 0], 2);
    a = metrics.pathMetrics(struct('pp', pp, 'xd', ppval(pp, [0, 1])));
    b = metrics.pathMetrics(struct('pp', pp, 'xd', ppval(pp, linspace(0, 1, 1001))));
    verifyEqual(testCase, a.meanCurvature, b.meanCurvature);
    verifyEqual(testCase, a.maxCurvature, b.maxCurvature);
    verifyEqual(testCase, a.pathLength, b.pathLength);
end

function testStopAtRightEndpointWithHeightRoundoff(testCase)
%TESTSTOPATRIGHTENDPOINTWITHHEIGHTROUNDOFF Остановка и шум постоянной высоты.
    pp = mkpp([0, 1], [0, 0, 0, -1, 3, -3, 1; ...
        1, -6, 15, -20, 15, -6, 1; 0, 0, 0, 0, 0, 1e-14, 10], 3);
    regular = mkpp([0, 1], [0, 1, 0; 1, 0, 0], 2);
    [meanValue, info] = metrics.calculateMeanCurvature(pp);
    verifyEqual(testCase, meanValue, metrics.calculateMeanCurvature(regular), 'AbsTol', 1e-8);
    verifyEmpty(testCase, info.cornerTimes);
    verifyEqual(testCase, metrics.calculateMaxCurvature(pp), 2, 'AbsTol', 1e-8);
end

function testHighOrderInteriorMaximum(testCase)
%TESTHIGHORDERINTERIORMAXIMUM Для (u,u^8) максимум лежит при u^14=1/160.
    pp = mkpp([0, 1], [zeros(1, 7), 1, 0; 1, zeros(1, 8)], 2);
    parameter = (1/160)^(1/14);
    expected = 56*parameter^6/(1 + 64*parameter^14)^(3/2);
    [maximum, info] = metrics.calculateMaxCurvature(pp);
    verifyEqual(testCase, maximum, expected, 'RelTol', 1e-8);
    verifyEqual(testCase, info.time, parameter, 'AbsTol', 1e-8);
end

function testProjectMinimumSnapEndpoint(testCase)
%TESTPROJECTMINIMUMSNAPENDPOINT Проверяет остановки полинома, построенного проектом.
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    state = rng;
    cleanup = onCleanup(@() rng(state));
    rng(121314);
    scenario = config.defaultScenario();
    [route, ~] = rrt.plan(scenario.start2d, scenario.goal2d, scenario.map, ...
        scenario.obstacles, scenario.safetyRadius, scenario.rrt);
    route = rrt.shortcutSmoothPath(route, scenario.map, scenario.obstacles, ...
        scenario.safetyRadius, scenario.rrt.smoothingIterations);
    trajectory = quadrotor.buildReferenceTrajectory(route, scenario.sim.vRef, scenario.sim.dt, scenario.sim.zRef);
    [meanValue, info] = metrics.calculateMeanCurvature(trajectory);
    verifyTrue(testCase, isfinite(meanValue));
    verifyGreaterThan(testCase, meanValue, 0);
    verifyEmpty(testCase, info.cornerTimes);
    verifyTrue(testCase, isfinite(metrics.calculateMaxCurvature(trajectory)));
    verifyEqual(testCase, trajectory.vd(:, [1, end]), zeros(3, 2));
end
