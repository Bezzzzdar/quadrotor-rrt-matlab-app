function result = isPointInObstacles(point, obstacles, safetyRadius)
%ISPOINTINOBSTACLES Проверяет попадание точки в препятствие с отступом.

    result = false;
    if isempty(obstacles)
        return;
    end

    x = point(1);
    y = point(2);

    for i = 1:size(obstacles, 1)
        ox = obstacles(i, 1) - safetyRadius;
        oy = obstacles(i, 2) - safetyRadius;
        ow = obstacles(i, 3) + 2 * safetyRadius;
        oh = obstacles(i, 4) + 2 * safetyRadius;

        if x >= ox && x <= ox + ow && y >= oy && y <= oy + oh
            result = true;
            return;
        end
    end
end
