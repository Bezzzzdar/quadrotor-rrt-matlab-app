function result = runPlannerExperiment(scenario, plannerName, simulateDynamics, planned)
%RUNPLANNEREXPERIMENT Запускает планировщик и общий расчёт траектории.
% В анализе четырёх критериев можно не рассчитывать ненужную динамику слежения.

    if nargin < 3, simulateDynamics = true; end

    geometry.validateScenario( ...
        scenario.map, scenario.start2d, scenario.goal2d, ...
        scenario.obstacles, scenario.safetyRadius);

    params = scenario.rrt;

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
    result.processedPath = [];
    result.path = [];
    result.referenceTrajectory = [];
    result.simLog = [];
    result.pathFound = ~isempty(path);
    result.calculationTime = calculationTime;
    result.message = '';
    result.status = 'no_path';
    result.trajectoryValidation = [];
    result.validationTime = 0;
    result.metrics = metrics.pathMetrics([]);

    if isempty(path)
        result.summary = makeSummary(result, []);
        result.message = 'Маршрут не найден.';
        if isfield(tree, 'message') && ~isempty(tree.message)
            result.message = tree.message;
        end
        return;
    end

    validationStart = tic;
    rawFree = geometry.isPathCollisionFree(path, scenario.map, scenario.obstacles, scenario.safetyRadius) && ...
        norm(path(1, :) - scenario.start2d) <= 1e-8 && norm(path(end, :) - scenario.goal2d) <= 1e-8;
    result.validationTime = toc(validationStart);
    if ~rawFree
        result.pathFound = false;
        result.status = 'invalid_path';
        result.message = 'Исходный маршрут недопустим: неверные концы или пересечение препятствия.';
        result.summary = makeSummary(result, []);
        return;
    end

    if plannerName == "rrt"
        simplifiedPath = path;
        smoothingIterations = params.smoothingIterations;
    else
        simplifiedPath = lineOfSightSimplifyPath( ...
            path, scenario.map, scenario.obstacles, scenario.safetyRadius);
        % Для A* и Вороного сохраняем середины сегментов, но исключаем случайные сокращения.
        smoothingIterations = 0;
    end

    smoothPath = rrt.shortcutSmoothPath( ...
        simplifiedPath, scenario.map, scenario.obstacles, scenario.safetyRadius, ...
        smoothingIterations);

    result.processedPath = smoothPath;
    result.referenceTrajectory = quadrotor.buildReferenceTrajectory( ...
        smoothPath, scenario.sim.vRef, scenario.sim.dt, scenario.sim.zRef);
    validationStart = tic;
    [free, result.trajectoryValidation] = geometry.isPolynomialTrajectoryFree( ...
        result.referenceTrajectory.pp, scenario.map, scenario.obstacles, scenario.safetyRadius);
    result.validationTime = result.validationTime + toc(validationStart);
    if ~free
        % Недопустимая траектория остаётся в диагностике, но не моделируется и не оценивается.
        result.pathFound = false;
        result.status = 'invalid_trajectory';
        result.message = result.trajectoryValidation.message;
        result.summary = makeSummary(result, []);
        return;
    end
    if simulateDynamics
        result.simLog = quadrotor.simulateOnPath( ...
            smoothPath, scenario.quad, scenario.ctrl, scenario.sim, result.referenceTrajectory);
    end
    result.status = 'solved';
    result.path = result.referenceTrajectory.xd(1:2, :)';
    result.metrics = metrics.pathMetrics(result.referenceTrajectory);
    result.summary = makeSummary(result, result.simLog);
    if ~simulateDynamics, result.summary.DynamicFeasible = NaN; end
    if simulateDynamics
        result.message = 'Маршрут найден, minimum-snap траектория построена, моделирование выполнено.';
    else
        result.message = 'Маршрут найден, minimum-snap траектория построена, критерии рассчитаны.';
    end
end

function simplifiedPath = lineOfSightSimplifyPath(path, map, obstacles, safetyRadius)
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
                    map, obstacles, safetyRadius)
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
