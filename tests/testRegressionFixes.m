function tests = testRegressionFixes
%TESTREGRESSIONFIXES Проверяет контрпримеры аудита и постоянную высоту полёта.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает код и сохраняет окружение.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.oldRng = rng;
    addpath(genpath(fullfile(root, 'src')));
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает окружение теста.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.oldRng);
end

function testThinObstacleAndClosedBoundary(testCase)
%TESTTHINOBSTACLEANDCLOSEDBOUNDARY Не пропускает препятствие между отсчётами.
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    obstacle = [4.2, 0, 0.1, 10];
    verifyFalse(testCase, geometry.isSegmentCollisionFree([0, 5], [10, 5], map, obstacle, 0));
    verifyFalse(testCase, geometry.isSegmentCollisionFree([0, 4], [10, 4], map, [4, 4, 2, 2], 0));
    verifyFalse(testCase, geometry.isSegmentCollisionFree([4, 4], [4, 4], map, [4, 4, 2, 2], 0));
    verifyTrue(testCase, geometry.isSegmentCollisionFree([0, 3], [10, 3], map, [4, 4, 2, 2], 0));
end

function testAStarThinWall(testCase)
%TESTASTARTHINWALL Не проводит внутреннее ребро сквозь стену между узлами.
    map = struct('width', 6, 'height', 2, 'resolution', 1);
    verifyEmpty(testCase, astar.plan([0, 1], [6, 1], map, [2.4, 0, 0.2, 2], 0, struct()));
end

function testAStarOffGridConnections(testCase)
%TESTASTAROFFGRIDCONNECTIONS Сохраняет свободный старт рядом с занятым узлом.
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    obstacle = [4, 4, 1, 2];
    route = astar.plan([3.8, 5], [1, 5], map, obstacle, 0, struct());
    assertNotEmpty(testCase, route);
    verifyEqual(testCase, route(1, :), [3.8, 5]);
    verifyEqual(testCase, route(end, :), [1, 5]);
    verifyTrue(testCase, geometry.isPathCollisionFree(route, map, obstacle, 0));
end

function testAStarEdgesOnFractionalMaps(testCase)
%TESTASTAREDGESONFRACTIONALMAPS Проверяет пакетные маски для дробных границ и препятствий.
    rng(151);
    map = struct('width', 10.4, 'height', 9.7, 'resolution', 1);
    for trial = 1:20
        obstacles = [2 + 5*rand(3, 2), 0.05 + 0.5*rand(3, 2)];
        route = astar.plan([0.2, 0.3], [10.2, 9.5], map, obstacles, 0, struct());
        assertNotEmpty(testCase, route);
        verifyTrue(testCase, geometry.isPathCollisionFree(route, map, obstacles, 0));
    end
end

function testInvalidGeometryRejected(testCase)
%TESTINVALIDGEOMETRYREJECTED Отклоняет NaN, Inf, отрицательный отступ и внешние препятствия.
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    verifyError(testCase, @() geometry.validateScenario(map, [1, 1], [9, 9], [NaN, 2, 1, 1], 0), ...
        'MATLAB:expectedFinite');
    verifyError(testCase, @() geometry.validateScenario(map, [1, 1], [9, 9], zeros(0, 4), -1), ...
        'MATLAB:expectedNonnegative');
    invalid = map;
    invalid.width = Inf;
    verifyError(testCase, @() geometry.validateScenario(invalid, [1, 1], [9, 9], zeros(0, 4), 0), ...
        'MATLAB:expectedFinite');
    verifyError(testCase, @() geometry.validateScenario(map, [1, 1], [9, 9], [9, 2, 2, 1], 0), ...
        'geometry:ObstacleOutsideMap');
end

function testObstacleUnionArea(testCase)
%TESTOBSTACLEUNIONAREA Не считает перекрывающиеся области несколько раз.
    map = struct('width', 1000, 'height', 1000, 'resolution', 1);
    geometry.validateScenario(map, [50, 50], [950, 950], ...
        [100, 100, 500, 400; 100, 100, 500, 400], 0);
    geometry.validateScenario(map, [50, 50], [950, 950], ...
        [100, 100, 500, 400; 350, 100, 500, 400], 0);
    verifyError(testCase, @() geometry.validateScenario(map, [50, 50], [950, 950], ...
        [100, 100, 500, 400; 400, 100, 500, 400], 0), 'geometry:ExcessObstacleArea');
end

function testStationaryGoalForAllMethods(testCase)
%TESTSTATIONARYGOALFORALLMETHODS Возвращает нулевую длину без исключений и фиктивной кривизны.
    scenario = config.defaultScenario();
    scenario.map = struct('width', 20, 'height', 20, 'resolution', 1);
    scenario.obstacles = zeros(0, 4);
    scenario.start2d = [3, 3];
    scenario.goal2d = scenario.start2d;
    for method = {'rrt', 'astar', 'voronoi'}
        result = runPlannerExperiment(scenario, method{1});
        verifyTrue(testCase, result.pathFound);
        verifyEqual(testCase, result.metrics.pathLength, 0);
        verifyTrue(testCase, isnan(result.metrics.maxCurvature));
        verifyTrue(testCase, isnan(result.metrics.meanCurvature));
        verifyEqual(testCase, result.simLog.trackingError, zeros(size(result.simLog.t)));
    end
end

function testRejectInvalidPreparedPath(testCase)
%TESTREJECTINVALIDPREPAREDPATH Не доверяет готовому маршруту с пересечением стены.
    scenario = config.defaultScenario();
    scenario.map = struct('width', 10, 'height', 10, 'resolution', 1);
    scenario.obstacles = [4.2, 0, 0.1, 10];
    scenario.safetyRadius = 0;
    scenario.start2d = [0, 5];
    scenario.goal2d = [10, 5];
    planned = struct('plannerName', 'astar', 'displayName', 'A*', ...
        'path', [0, 5; 10, 5], 'tree', [], 'calculationTime', 0);
    result = runPlannerExperiment(scenario, 'astar', true, planned);
    verifyFalse(testCase, result.pathFound);
    verifyEqual(testCase, result.status, 'invalid_path');
    verifyEmpty(testCase, result.simLog);
    verifyTrue(testCase, isnan(result.metrics.pathLength));
end

function testSpatialCurvatureRotation(testCase)
%TESTSPATIALCURVATUREROTATION Поворот параболы из xy в xz сохраняет её кривизну.
    planar = mkpp([0, 1], [0, 1, 0; 1, 0, 0], 2);
    spatial = mkpp([0, 1], [0, 1, 0; 0, 0, 0; 1, 0, 0], 3);
    a = metrics.pathMetrics(planar);
    b = metrics.pathMetrics(spatial);
    verifyEqual(testCase, b.maxCurvature, a.maxCurvature, 'AbsTol', 1e-12);
    verifyEqual(testCase, b.meanCurvature, a.meanCurvature, 'AbsTol', 1e-12);
end

function testRejectUnsafeMinimumSnapBeforeSimulation(testCase)
%TESTREJECTUNSAFEMINIMUMSNAPBEFORESIMULATION Не моделирует полином, срезающий безопасный угол.
    scenario = smallScenario();
    scenario.rrt.smoothingIterations = 0;
    points = [3, 3; 3, 31.1; 21, 31.1; 37, 37];
    planned = struct('plannerName', 'rrt', 'displayName', 'RRT', ...
        'path', points, 'tree', [], 'calculationTime', 0);
    result = runPlannerExperiment(scenario, 'rrt', true, planned);
    verifyTrue(testCase, geometry.isPathCollisionFree(result.processedPath, ...
        scenario.map, scenario.obstacles, scenario.safetyRadius));
    verifyFalse(testCase, geometry.isPolynomialTrajectoryFree(result.referenceTrajectory.pp, ...
        scenario.map, scenario.obstacles, scenario.safetyRadius));
    verifyFalse(testCase, result.pathFound);
    verifyEqual(testCase, result.status, 'invalid_trajectory');
    verifyEmpty(testCase, result.simLog);
    verifyEmpty(testCase, result.path);
    verifyTrue(testCase, isnan(result.metrics.pathLength));
end

function testAnalysisExcludesUnsafeTrajectory(testCase)
%TESTANALYSISEXCLUDESUNSAFETRAJECTORY Не усредняет критерии недопустимой minimum-snap.
    scenario = smallScenario();
    variants.stepSize = struct('value', 5, 'label', 'Шаг');
    result = analysis.runMethod(scenario, 'rrt', variants, 1, 47);
    verifyEqual(testCase, result.tables.stepSize.PathFoundRate, 0);
    verifyEqual(testCase, result.tables.stepSize.ErrorCount, 0);
    verifyTrue(testCase, isnan(result.tables.stepSize.MeanPathLength_m));
    verifyNotEmpty(testCase, char(result.tables.stepSize.Message));
end

function testConstantAltitudeAndAllDerivatives(testCase)
%TESTCONSTANTALTITUDEANDALLDERIVATIVES Проверяет постоянный z в полиноме и движении ЛА.
    scenario = config.defaultScenario();
    points = [0, 0; 20, 0; 30, 20; 50, 20];
    [log, trajectory] = quadrotor.simulateOnPath(points, scenario.quad, scenario.ctrl, scenario.sim);
    verifyEqual(testCase, log.motionModel, 'planar_constant_altitude');
    verifyEqual(testCase, log.x(3, :), scenario.sim.zRef*ones(size(log.t)));
    values = ppval(trajectory.pp, linspace(trajectory.t(1), trajectory.t(end), 1001));
    verifyEqual(testCase, values(3, :), scenario.sim.zRef*ones(1, 1001));
    derivatives = {log.firstDerivative, log.secondDerivative, log.thirdDerivative, log.fourthDerivative, ...
        trajectory.vd, trajectory.ad, trajectory.jerk, trajectory.snap};
    for index = 1:numel(derivatives)
        verifyEqual(testCase, derivatives{index}(3, :), zeros(size(log.t)));
    end
end

function testSaturationCannotPassFeasibility(testCase)
%TESTSATURATIONCANNOTPASSFEASIBILITY Неподвижность не скрывает нехватку тяги для удержания высоты.
    scenario = config.defaultScenario();
    scenario.quad.fMaxPerMotor = 1;
    log = quadrotor.simulateOnPath([0, 0], scenario.quad, scenario.ctrl, scenario.sim);
    verifyTrue(testCase, any(log.motorSaturated));
    verifyGreaterThan(testCase, max(log.requestedMotorThrusts, [], 'all'), scenario.quad.fMaxPerMotor);
    verifyEqual(testCase, log.maxTrackingError, 0);
    verifyFalse(testCase, log.dynamicFeasible);
    verifyEqual(testCase, log.x(3, :), scenario.sim.zRef*ones(size(log.t)));
end

function testMaximumTrackingErrorThreshold(testCase)
%TESTMAXIMUMTRACKINGERRORTHRESHOLD Проверяет максимум за весь полёт вместо конечной ошибки.
    scenario = config.defaultScenario();
    points = [0, 0; 20, 0; 30, 20; 50, 20];
    [baseline, trajectory] = quadrotor.simulateOnPath(points, scenario.quad, scenario.ctrl, scenario.sim);
    assertGreaterThan(testCase, baseline.maxTrackingError, baseline.finalTrackingError);
    scenario.sim.maxTrackingError = (baseline.maxTrackingError + baseline.finalTrackingError)/2;
    log = quadrotor.simulateOnPath(points, scenario.quad, scenario.ctrl, scenario.sim, trajectory);
    verifyLessThan(testCase, log.finalTrackingError, scenario.sim.maxTrackingError);
    verifyGreaterThan(testCase, log.maxTrackingError, scenario.sim.maxTrackingError);
    verifyFalse(testCase, log.dynamicFeasible);
end

function testAnalysisExcelPreservesSignedInfinity(testCase)
%TESTANALYSISEXCELPRESERVESSIGNEDINFINITY Проверяет три книги, три листа и обе бесконечности.
    folder = tempname;
    mkdir(folder);
    cleanup = onCleanup(@() removeTemporaryFolder(folder));
    for method = {'rrt', 'astar', 'voronoi'}
        for parameter = {'first', 'second', 'third'}
            name = parameter{1};
            results.(method{1}).tables.(name) = table([1; 2; 3], [Inf; -Inf; NaN], ...
                'VariableNames', {'ParameterValue', 'Curvature'});
            variants.(method{1}).(name).label = ['Проверка ', name];
        end
    end
    analysis.exportAnalysis(results, variants, folder);
    analysis.exportAnalysis(results, variants, folder);
    for method = {'rrt', 'astar', 'voronoi'}
        file = fullfile(folder, ['analysis_', method{1}, '.xlsx']);
        verifyEqual(testCase, sheetnames(file), ["first"; "second"; "third"]);
        for parameter = {'first', 'second', 'third'}
            name = parameter{1};
            values = readcell(file, 'Sheet', name, 'Range', 'B4:B5');
            verifyEqual(testCase, string(values), ["Inf"; "-Inf"]);
            heading = readcell(file, 'Sheet', name, 'Range', 'A1:A1');
            verifyTrue(testCase, contains(string(heading{1}), ['Проверка ', name]));
        end
    end
end

function removeTemporaryFolder(folder)
%REMOVETEMPORARYFOLDER Удаляет только созданную тестом папку системного temp.
    if isfolder(folder) && startsWith([folder, filesep], tempdir, 'IgnoreCase', true)
        rmdir(folder, 's');
    end
end

function scenario = smallScenario()
%SMALLSCENARIO Возвращает карту контрпримера для проверки сглаживания.
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
end
