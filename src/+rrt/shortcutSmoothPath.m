function smoothPath = shortcutSmoothPath(path, map, obstacles, safetyRadius, checkStep, iterations)
%SHORTCUTSMOOTHPATH Сглаживает маршрут методом случайных сокращений.

smoothPath = path;

if size(smoothPath, 1) <= 2
    return;
end

for k = 1:iterations
    n = size(smoothPath, 1);

    if n <= 2
        return;
    end

    i = randi([1, n - 1]);
    j = randi([i + 1, n]);

    if j <= i + 1
        continue;
    end

    p1 = smoothPath(i, :);
    p2 = smoothPath(j, :);

    if geometry.isSegmentCollisionFree(p1, p2, map, obstacles, safetyRadius, checkStep)
        smoothPath = [smoothPath(1:i, :); smoothPath(j:end, :)]; %#ok<AGROW>
    end
end
end
