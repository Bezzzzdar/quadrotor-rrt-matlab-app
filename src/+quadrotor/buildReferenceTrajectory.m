function traj = buildReferenceTrajectory(path2d, vRef, dt, zRef)
%BUILDREFERENCETRAJECTORY Строит minimum-snap 3D-траекторию по 2D-маршруту.
% Разгон и торможение выполняются по прямым с нулевой кривизной.
% На концах скорость равна нулю; стыки согласованы по производным до snap.
% Литература: https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
% Граничные условия: https://www.mathworks.com/help/fusion/ref/minsnappolytraj.html

    validateattributes(path2d, {'numeric'}, {'2d', 'real', 'finite', 'ncols', 2, 'nonempty'});
    validateattributes(vRef, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    validateattributes(dt, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    validateattributes(zRef, {'numeric'}, {'scalar', 'real', 'finite'});

    if exist('minsnappolytraj', 'file') ~= 2
        error('Функция minsnappolytraj не найдена. Требуется MATLAB toolbox с этой функцией.');
    end

    segmentLengths2d = sqrt(sum(diff(path2d, 1, 1).^2, 2));
    path2d = path2d([true; segmentLengths2d > eps], :);

    points = [path2d, zRef * ones(size(path2d, 1), 1)]';
    if size(points, 2) == 1
        % Уже достигнутая цель задаёт неподвижную траекторию на одном шаге времени.
        pp = mkpp([0, dt], points, 3);
        waypointTimes = [0, dt];
        points = [points, points];
    elseif size(points, 2) == 2
        duration = max(2 * norm(points(:, end) - points(:, 1)) / vRef, dt);
        pp = straightStopSegment(points(:, 1), points(:, end), duration, 0, 0);
        waypointTimes = [0, duration];
    else
        % Четверть крайних отрезков резервируется под прямолинейные фазы.
        fraction = 0.25;
        startDisplacement = fraction * (points(:, 2) - points(:, 1));
        endDisplacement = fraction * (points(:, end) - points(:, end - 1));
        startDuration = max(2 * norm(startDisplacement) / vRef, dt);
        endDuration = max(2 * norm(endDisplacement) / vRef, dt);
        startVelocity = 2 * startDisplacement / startDuration;
        endVelocity = 2 * endDisplacement / endDuration;

        middlePoints = [points(:, 1) + startDisplacement, ...
            points(:, 2:end - 1), points(:, end) - endDisplacement];
        lengths = sqrt(sum(diff(middlePoints, 1, 2).^2, 1));
        middleTimes = [0, cumsum(max(lengths / vRef, dt))];
        velocityConditions = nan(size(middlePoints));
        velocityConditions(:, 1) = startVelocity;
        velocityConditions(:, end) = endVelocity;
        middlePP = solveMinimumSnap(middlePoints, middleTimes, velocityConditions);

        startPP = straightStopSegment(points(:, 1), middlePoints(:, 1), ...
            startDuration, 0, norm(startVelocity));
        endPP = straightStopSegment(middlePoints(:, end), points(:, end), ...
            endDuration, norm(endVelocity), 0);
        pp = concatenatePhases(startPP, middlePP, endPP);
        waypointTimes = [0, startDuration + middleTimes, pp.breaks(end)];
        points = [points(:, 1), middlePoints, points(:, end)];
    end

    % Высота задаётся точно постоянной и в полиноме, и в отсчётах.
    pp.coefs(3:3:end, :) = 0;
    pp.coefs(3:3:end, end) = zRef;
    numSamples = max(2, ceil(pp.breaks(end) / dt) + 1);
    t = linspace(0, pp.breaks(end), numSamples);
    samples = sampleDerivatives(pp, t);
    % Устанавливаются точно заданные условия покоя, без ошибки округления ppval.
    samples{1}(:, 1) = points(:, 1);
    samples{1}(:, end) = points(:, end);
    for derivative = 2:5
        samples{derivative}(:, [1, end]) = 0;
    end

    traj.t = t;
    traj.timePoints = waypointTimes;
    traj.waypoints = points;
    traj.xd = samples{1};
    traj.vd = samples{2};
    traj.ad = samples{3};
    traj.jerk = samples{4};
    traj.snap = samples{5};
    traj.pp = pp;
end

function pp = straightStopSegment(startPoint, endPoint, duration, startSpeed, endSpeed)
%STRAIGHTSTOPSEGMENT Минимизирует snap скалярного продвижения по прямой.
% Граничные условия: https://www.mathworks.com/help/fusion/ref/minsnappolytraj.html
    displacement = endPoint - startPoint;
    distance = norm(displacement);
    scalarPP = solveMinimumSnap([0, distance], [0, duration], [startSpeed, endSpeed]);
    direction = displacement / distance;
    coefficients = direction * scalarPP.coefs;
    coefficients(:, end) = coefficients(:, end) + startPoint;
    pp = mkpp(scalarPP.breaks, coefficients, 3);
end

function pp = solveMinimumSnap(points, timePoints, velocityConditions)
%SOLVEMINIMUMSNAP Нормализует время для устойчивого расчёта коротких сегментов.
% Постоянное масштабирование времени сохраняет минимум интеграла snap^2.
% Литература: https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
    timeScale = min(diff(timePoints));
    [~, ~, ~, ~, ~, normalizedPP] = minsnappolytraj( ...
        points, timePoints / timeScale, 2, ...
        'VelocityBoundaryCondition', velocityConditions * timeScale);
    powers = normalizedPP.order - 1:-1:0;
    coefficients = normalizedPP.coefs ./ timeScale.^powers;
    pp = mkpp(normalizedPP.breaks * timeScale, coefficients, size(points, 1));
end

function pp = concatenatePhases(startPP, middlePP, endPP)
%CONCATENATEPHASES Объединяет фазы в один кусочно-полиномиальный путь.
% Согласование производных: https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
    order = max([startPP.order, middlePP.order, endPP.order]);
    blocks = {startPP.coefs, middlePP.coefs, endPP.coefs};
    for index = 1:numel(blocks)
        blocks{index} = [zeros(size(blocks{index}, 1), order - size(blocks{index}, 2)), blocks{index}];
    end
    startTime = startPP.breaks(end);
    endTime = startTime + middlePP.breaks(end);
    breaks = [startPP.breaks(1:end - 1), ...
        startTime + middlePP.breaks, endTime + endPP.breaks(2:end)];
    pp = mkpp(breaks, vertcat(blocks{:}), 3);
end

function samples = sampleDerivatives(pp, time)
%SAMPLEDERIVATIVES Вычисляет положение и аналитические производные полинома.
% Представление траектории: https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
    samples = cell(1, 5);
    coefficients = pp.coefs;
    for derivative = 1:5
        samples{derivative} = ppval(mkpp(pp.breaks, coefficients, 3), time);
        if size(coefficients, 2) > 1
            coefficients = coefficients(:, 1:end - 1) .* (size(coefficients, 2) - 1:-1:1);
        else
            coefficients = zeros(size(coefficients, 1), 1);
        end
    end
end
