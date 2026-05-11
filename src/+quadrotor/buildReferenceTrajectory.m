function traj = buildReferenceTrajectory(path2d, vRef, dt, zRef)
%BUILDREFERENCETRAJECTORY Строит minimum-snap 3D-траекторию по 2D-маршруту.

    if size(path2d, 1) < 2
        error('Для построения траектории требуется минимум две точки.');
    end

    if vRef <= 0
        error('Опорная скорость должна быть положительной.');
    end

    if dt <= 0
        error('Шаг интегрирования должен быть положительным.');
    end

    if exist('minsnappolytraj', 'file') ~= 2
        error('Функция minsnappolytraj не найдена. Требуется MATLAB toolbox с этой функцией.');
    end

    segmentLengths2d = sqrt(sum(diff(path2d, 1, 1).^2, 2));
    path2d = path2d([true; segmentLengths2d > eps], :);

    if size(path2d, 1) < 2
        error('Для построения траектории требуется минимум две различные точки.');
    end

    points = [path2d, zRef * ones(size(path2d, 1), 1)]';
    segmentLengths = sqrt(sum(diff(points, 1, 2).^2, 1));
    segmentTimes = max(segmentLengths / vRef, dt);
    timePoints = [0, cumsum(segmentTimes)];
    numSamples = max(2, ceil(timePoints(end) / dt) + 1);

    [xd, vd, ad, jerk, snap, pp, waypointTimes, t] = minsnappolytraj( ...
        points, timePoints, numSamples);

    traj.t = t;
    traj.timePoints = waypointTimes;
    traj.waypoints = points;
    traj.xd = xd;
    traj.vd = vd;
    traj.ad = ad;
    traj.jerk = jerk;
    traj.snap = snap;
    traj.pp = pp;
end

% function traj = buildReferenceTrajectory(path2d, vRef, dt, zRef)
% %BUILDREFERENCETRAJECTORY Builds a 3D reference trajectory from a 2D path.

% if size(path2d, 1) < 2
%     error('Для построения траектории требуется минимум две точки.');
% end

% if vRef <= 0
%     error('Опорная скорость должна быть положительной.');
% end

% if dt <= 0
%     error('Шаг интегрирования должен быть положительным.');
% end

% if exist('minsnappolytraj', 'file') ~= 2
%     error('Функция minsnappolytraj не найдена. Требуется MATLAB toolbox с этой функцией.');
% end

% path2d = removeRepeatedPoints(path2d);

% if size(path2d, 1) < 2
%     error('Для построения траектории требуется минимум две различные точки.');
% end

% points = [path2d, zRef * ones(size(path2d, 1), 1)]';
% isTurn = detectTurns(points);

% traj = emptyTrajectory(points);
% pointIndex = 1;

% while pointIndex < size(points, 2)
%     if pointIndex <= size(points, 2) - 2 && isTurn(pointIndex + 1)
%         chunk = buildSnapTurnChunk(points(:, pointIndex:pointIndex + 2), vRef, dt);
%         pointIndex = pointIndex + 2;
%     else
%         chunk = buildLinearChunk(points(:, pointIndex), points(:, pointIndex + 1), vRef, dt);
%         pointIndex = pointIndex + 1;
%     end

%     traj = appendChunk(traj, chunk);
% end

% traj.timePoints = [];
% traj.waypoints = points;
% traj.pp = [];
% end

% function path2d = removeRepeatedPoints(path2d)
% segmentLengths = sqrt(sum(diff(path2d, 1, 1).^2, 2));
% path2d = path2d([true; segmentLengths > eps], :);
% end

% function isTurn = detectTurns(points)
% turnAngleThreshold = 5 * pi / 180;
% isTurn = false(1, size(points, 2));

% for i = 2:size(points, 2) - 1
%     prevVector = points(:, i) - points(:, i - 1);
%     nextVector = points(:, i + 1) - points(:, i);

%     if norm(prevVector) < eps || norm(nextVector) < eps
%         continue;
%     end

%     prevDirection = prevVector / norm(prevVector);
%     nextDirection = nextVector / norm(nextVector);
%     cosAngle = max(-1, min(1, dot(prevDirection, nextDirection)));
%     turnAngle = acos(cosAngle);
%     isTurn(i) = turnAngle > turnAngleThreshold;
% end
% end

% function chunk = buildLinearChunk(p1, p2, vRef, dt)
% segmentLength = norm(p2 - p1);
% duration = max(segmentLength / vRef, dt);
% numSamples = max(2, ceil(duration / dt) + 1);
% time = linspace(0, duration, numSamples);
% tau = time / duration;
% direction = (p2 - p1) / max(segmentLength, eps);

% chunk.t = time;
% chunk.xd = p1 + (p2 - p1) .* tau;
% chunk.vd = repmat(vRef * direction, 1, numSamples);
% chunk.ad = zeros(3, numSamples);
% chunk.jerk = zeros(3, numSamples);
% chunk.snap = zeros(3, numSamples);
% end

% function chunk = buildSnapTurnChunk(turnPoints, vRef, dt)
% segmentLengths = sqrt(sum(diff(turnPoints, 1, 2).^2, 1));
% segmentTimes = max(segmentLengths / vRef, dt);
% timePoints = [0, cumsum(segmentTimes)];
% numSamples = max(3, ceil(timePoints(end) / dt) + 1);

% velocityBoundary = nan(size(turnPoints));
% velocityBoundary(:, 1) = vRef * unitVector(turnPoints(:, 2) - turnPoints(:, 1));
% velocityBoundary(:, end) = vRef * unitVector(turnPoints(:, end) - turnPoints(:, end - 1));

% [xd, vd, ad, jerk, snap, ~, ~, tSamples] = minsnappolytraj( ...
%     turnPoints, timePoints, numSamples, ...
%     VelocityBoundaryCondition=velocityBoundary);

% chunk.t = tSamples;
% chunk.xd = xd;
% chunk.vd = vd;
% chunk.ad = ad;
% chunk.jerk = jerk;
% chunk.snap = snap;
% end

% function direction = unitVector(vector)
% direction = vector / max(norm(vector), eps);
% end

% function traj = emptyTrajectory(points)
% traj.t = [];
% traj.xd = zeros(size(points, 1), 0);
% traj.vd = zeros(size(points, 1), 0);
% traj.ad = zeros(size(points, 1), 0);
% traj.jerk = zeros(size(points, 1), 0);
% traj.snap = zeros(size(points, 1), 0);
% end

% function traj = appendChunk(traj, chunk)
% if isempty(traj.t)
%     traj.t = chunk.t;
%     traj.xd = chunk.xd;
%     traj.vd = chunk.vd;
%     traj.ad = chunk.ad;
%     traj.jerk = chunk.jerk;
%     traj.snap = chunk.snap;
%     return;
% end

% timeOffset = traj.t(end);
% traj.t = [traj.t, timeOffset + chunk.t(2:end)];
% traj.xd = [traj.xd, chunk.xd(:, 2:end)];
% traj.vd = [traj.vd, chunk.vd(:, 2:end)];
% traj.ad = [traj.ad, chunk.ad(:, 2:end)];
% traj.jerk = [traj.jerk, chunk.jerk(:, 2:end)];
% traj.snap = [traj.snap, chunk.snap(:, 2:end)];
% end
