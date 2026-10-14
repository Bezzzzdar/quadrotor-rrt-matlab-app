function result = isSegmentCollisionFree(p1, p2, map, obstacles, safetyRadius, checkStep)
%ISSEGMENTCOLLISIONFREE Проверяет отрезок на пересечение с препятствиями.

    validateattributes(checkStep, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    segmentLength = norm(p2 - p1);
    n = max(2, ceil(segmentLength / checkStep));
    points = p1 + ((0:n)' / n) .* (p2 - p1);
    result = all(points(:, 1) >= 0 & points(:, 1) <= map.width & ...
        points(:, 2) >= 0 & points(:, 2) <= map.height);
    if ~result
        return;
    end

    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        if any(points(:, 1) >= obstacle(1) - safetyRadius & ...
                points(:, 1) <= obstacle(1) + obstacle(3) + safetyRadius & ...
                points(:, 2) >= obstacle(2) - safetyRadius & ...
                points(:, 2) <= obstacle(2) + obstacle(4) + safetyRadius)
            result = false;
            return;
        end
    end
end
