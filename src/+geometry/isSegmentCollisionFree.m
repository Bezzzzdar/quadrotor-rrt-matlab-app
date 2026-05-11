function result = isSegmentCollisionFree(p1, p2, map, obstacles, safetyRadius, checkStep)
%ISSEGMENTCOLLISIONFREE Проверяет отрезок на пересечение с препятствиями.

    result = true;
    segmentLength = norm(p2 - p1);
    n = max(2, ceil(segmentLength / checkStep));

    for i = 0:n
        t = i / n;
        p = p1 + t * (p2 - p1);

        if ~geometry.isPointInsideMap(p, map) || ...
        geometry.isPointInObstacles(p, obstacles, safetyRadius)
            result = false;
            return;
        end
    end
end
