function traj = buildReferenceTrajectory(path2d, vRef, dt, zRef)
%BUILDREFERENCETRAJECTORY Строит дискретную 3D-траекторию по 2D-маршруту.

if size(path2d, 1) < 2
    error('Для построения траектории требуется минимум две точки.');
end

points = [path2d, zRef * ones(size(path2d, 1), 1)]';
segmentLengths = sqrt(sum(diff(points, 1, 2).^2, 1));
segmentTimes = segmentLengths / vRef;

xd = [];
vd = [];
ad = [];
t = [];
timeNow = 0;

for i = 1:length(segmentLengths)
    p1 = points(:, i);
    p2 = points(:, i + 1);
    T = max(segmentTimes(i), dt);
    n = max(2, ceil(T / dt));
    direction = (p2 - p1) / max(segmentLengths(i), eps);

    for k = 0:n-1
        tau = k / n;
        xd = [xd, p1 + tau * (p2 - p1)]; %#ok<AGROW>
        vd = [vd, vRef * direction]; %#ok<AGROW>
        ad = [ad, zeros(3, 1)]; %#ok<AGROW>
        t = [t, timeNow]; %#ok<AGROW>
        timeNow = timeNow + dt;
    end
end

xd = [xd, points(:, end)];
vd = [vd, zeros(3, 1)];
ad = [ad, zeros(3, 1)];
t = [t, timeNow];

traj.t = t;
traj.xd = xd;
traj.vd = vd;
traj.ad = ad;
end
