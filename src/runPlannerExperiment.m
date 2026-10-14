function result = runPlannerExperiment(scenario, plannerName, simulateDynamics, planned)
%RUNPLANNEREXPERIMENT Запускает планировщик и общий расчёт траектории.
% В анализе четырёх критериев можно не рассчитывать ненужную динамику слежения.

    if nargin < 3, simulateDynamics = true; end

    geometry.validateScenario( ...
        scenario.map, scenario.start2d, scenario.goal2d, ...
        scenario.obstacles, scenario.safetyRadius);

    params = scenario.rrt;
    if params.goalThreshold <= 0
        params.goalThreshold = params.stepSize;
    end

    plannerName = lower(string(plannerName));

    if nargin < 4
        planned = planGeometricPath(scenario, plannerName);
    elseif ~strcmp(planned.plannerName, plannerName)
        error('planning:PlannerMismatch', 'Готовый маршрут относится к другому планировщику.');
    end
    path = planned.path;
    tree = planned.tree;
    displayName = planned.displayName;
    calculationTime = planned.calculationTime;

    result.plannerName = char(plannerName);
    result.displayName = char(displayName);
    result.tree = tree;
    result.rawPath = path;
    result.rrtPath = [];
    result.path = [];
    result.referenceTrajectory = [];
    result.simLog = [];
    result.pathFound = ~isempty(path);
    result.calculationTime = calculationTime;
    result.message = '';

    if isempty(path)
        result.metrics = emptyMetrics();
        result.summary = makeSummary(result, []);
        result.message = 'Маршрут не найден.';
        if isfield(tree, 'message') && ~isempty(tree.message)
            result.message = tree.message;
        end
        return;
    end

    if plannerName == "rrt"
        simplifiedPath = path;
        smoothingIterations = params.smoothingIterations;
    else
        simplifiedPath = lineOfSightSimplifyPath( ...
            path, scenario.map, scenario.obstacles, scenario.safetyRadius, ...
            params.collisionCheckStep);
        % Для A* и Вороного сохраняем середины сегментов, но исключаем случайные сокращения.
        smoothingIterations = 0;
    end

    smoothPath = rrt.shortcutSmoothPath( ...
        simplifiedPath, scenario.map, scenario.obstacles, scenario.safetyRadius, ...
        params.collisionCheckStep, smoothingIterations);

    result.rrtPath = smoothPath;
    if simulateDynamics
        [result.simLog, result.referenceTrajectory] = quadrotor.simulateOnPath( ...
            smoothPath, scenario.quad, scenario.ctrl, scenario.sim);
    else
        result.referenceTrajectory = quadrotor.buildReferenceTrajectory( ...
            smoothPath, scenario.sim.vRef, scenario.sim.dt, scenario.sim.zRef);
    end
    result.path = result.referenceTrajectory.xd(1:2, :)';
    result.metrics = metrics.pathMetrics(result.path, params.densePathStep, result.referenceTrajectory);
    result.summary = makeSummary(result, result.simLog);
    if ~simulateDynamics, result.summary.DynamicFeasible = NaN; end
    if simulateDynamics
        result.message = 'Маршрут найден, minimum-snap траектория построена, моделирование выполнено.';
    else
        result.message = 'Маршрут найден, minimum-snap траектория построена, критерии рассчитаны.';
    end
end

function simplifiedPath = lineOfSightSimplifyPath(path, map, obstacles, safetyRadius, checkStep)
%LINEOFSIGHTSIMPLIFYPATH Убирает избыточные точки с проверкой препятствий.

    if size(path, 1) <= 2
        simplifiedPath = path;
        return;
    end

    indices = zeros(size(path, 1), 1);
    count = 1;
    indices(count) = 1;
    currentIndex = 1;

    while currentIndex < size(path, 1)
        nextIndex = currentIndex + 1;

        for candidateIndex = size(path, 1):-1:currentIndex + 1
            if geometry.isSegmentCollisionFree( ...
                    path(currentIndex, :), path(candidateIndex, :), ...
                    map, obstacles, safetyRadius, checkStep)
                nextIndex = candidateIndex;
                break;
            end
        end

        count = count + 1;
        indices(count) = nextIndex;
        currentIndex = nextIndex;
    end
    simplifiedPath = path(indices(1:count), :);
end

function m = emptyMetrics()
%EMPTYMETRICS Возвращает метрики при отсутствии маршрута.

    m = struct( ...
        'pathLength', NaN, ...
        'maxCurvature', NaN, ...
        'meanCurvature', NaN, ...
        'densePath', []);
end

function summary = makeSummary(result, simLog)
%MAKESUMMARY Формирует строку таблицы результатов.

    if isempty(simLog)
        dynamicFeasible = false;
        finalTrackingError = NaN;
        maxTrackingError = NaN;
        maxMotorThrust = NaN;
        meanMotorThrust = NaN;
    else
        dynamicFeasible = simLog.dynamicFeasible;
        finalTrackingError = simLog.finalTrackingError;
        maxTrackingError = simLog.maxTrackingError;
        maxMotorThrust = simLog.maxMotorThrust;
        meanMotorThrust = simLog.meanMotorThrust;
    end

    summary = table( ...
        result.pathFound, ...
        result.calculationTime, ...
        result.metrics.pathLength, ...
        result.metrics.maxCurvature, ...
        result.metrics.meanCurvature, ...
        dynamicFeasible, ...
        finalTrackingError, ...
        maxTrackingError, ...
        maxMotorThrust, ...
        meanMotorThrust, ...
        'VariableNames', { ...
        'PathFound', ...
        'CalculationTime_s', ...
        'PathLength_m', ...
        'MaxCurvature_1_m', ...
        'MeanCurvature_1_m', ...
        'DynamicFeasible', ...
        'FinalTrackingError_m', ...
        'MaxTrackingError_m', ...
        'MaxMotorThrust_N', ...
        'MeanMotorThrust_N'});
end
