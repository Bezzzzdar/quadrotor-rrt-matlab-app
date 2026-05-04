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
