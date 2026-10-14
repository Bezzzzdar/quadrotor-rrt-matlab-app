function tests = testReferenceTrajectory
%TESTREFERENCETRAJECTORY Проверяет остановки и стыки minimum-snap фаз.
% Литература: https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает исходные функции и проверяет зависимость MATLAB.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    addpath(genpath(fullfile(root, 'src')));
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает путь MATLAB.
    path(testCase.TestData.oldPath);
end

function testDiagonalStraightPath(testCase)
%TESTDIAGONALSTRAIGHTPATH Прямая остаётся прямой при разгоне и полной остановке.
    points = [3, 7; 103, 57];
    trajectory = quadrotor.buildReferenceTrajectory(points, 10, 0.02, 8);
    verifyEqual(testCase, trajectory.vd(:, [1, end]), zeros(3, 2));
    verifyEqual(testCase, metrics.calculateMaxCurvature(trajectory), 0);
    verifyEqual(testCase, metrics.calculateMeanCurvature(trajectory), 0);
    verifyEqual(testCase, trajectory.xd(1:2, [1, end])', points);
    direction = (points(2, :) - points(1, :))';
    projection = direction' * trajectory.vd(1:2, :);
    verifyGreaterThanOrEqual(testCase, min(projection), -1e-8);
end

function testCurvedPathAndDerivativeContinuity(testCase)
%TESTCURVEDPATHANDDERIVATIVECONTINUITY На стыках непрерывны производные до snap.
    points = [0, 0; 40, 0; 80, 30; 120, 30];
    trajectory = quadrotor.buildReferenceTrajectory(points, 5, 0.02, 10);
    verifyEqual(testCase, trajectory.vd(:, [1, end]), zeros(3, 2));
    verifyEqual(testCase, trajectory.ad(:, [1, end]), zeros(3, 2));
    verifyTrue(testCase, isfinite(metrics.calculateMaxCurvature(trajectory)));
    pp = trajectory.pp;
    for piece = 1:pp.pieces - 1
        duration = pp.breaks(piece + 1) - pp.breaks(piece);
        for derivative = 0:4
            left = evaluatePiece(pp, piece, duration, derivative);
            right = evaluatePiece(pp, piece + 1, 0, derivative);
            verifyEqual(testCase, left, right, 'AbsTol', 1e-7);
        end
    end
    positions = ppval(pp, trajectory.timePoints);
    verifyEqual(testCase, positions, trajectory.waypoints, 'AbsTol', 1e-7);
    verifyEqual(testCase, trajectory.xd(3, :), 10*ones(size(trajectory.t)), 'AbsTol', 1e-9);
end

function testOuterPhasesStayOnOriginalSegments(testCase)
%TESTOUTERPHASESSTAYONORIGINALSEGMENTS Прямые фазы не выходят за крайние отрезки.
    points = [0, 0; 40, 10; 70, 60];
    trajectory = quadrotor.buildReferenceTrajectory(points, 5, 0.02, 10);
    pp = trajectory.pp;
    for piece = [1, pp.pieces]
        if piece == 1
            a = points(1, :)';
            b = points(2, :)';
        else
            a = points(end - 1, :)';
            b = points(end, :)';
        end
        sampled = ppval(pp, linspace(pp.breaks(piece), pp.breaks(piece + 1), 101));
        direction = b - a;
        fraction = direction' * (sampled(1:2, :) - a) / norm(direction)^2;
        residual = sampled(1:2, :) - (a + direction * fraction);
        verifyLessThan(testCase, max(abs(residual(:))), 1e-8);
        verifyGreaterThanOrEqual(testCase, min(fraction), -1e-8);
        verifyLessThanOrEqual(testCase, max(fraction), 1 + 1e-8);
    end
end

function testShortSegmentsAndRepeatedPoints(testCase)
%TESTSHORTSEGMENTSANDREPEATEDPOINTS Проверяет короткие отрезки и совпадающие точки.
    trajectory = quadrotor.buildReferenceTrajectory([0, 0; 0, 0; 0.01, 0; 0.02, 0.01], 10, 0.02, 1);
    verifyEqual(testCase, trajectory.vd(:, [1, end]), zeros(3, 2));
    verifyTrue(testCase, isfinite(metrics.calculateMaxCurvature(trajectory)));
    verifyGreaterThan(testCase, min(diff(trajectory.t)), 0);
    verifyLessThanOrEqual(testCase, max(diff(trajectory.t)), 0.02 + 1e-12);
end

function value = evaluatePiece(pp, piece, time, derivative)
%EVALUATEPIECE Вычисляет односторонние значения производных сегмента.
% Представление: https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
    coefficients = pp.coefs((piece - 1)*3 + (1:3), :);
    for order = 1:derivative
        coefficients = coefficients(:, 1:end - 1) .* (size(coefficients, 2) - 1:-1:1);
    end
    value = zeros(3, 1);
    for component = 1:3
        value(component) = polyval(coefficients(component, :), time);
    end
end
