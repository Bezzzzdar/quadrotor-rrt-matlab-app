function validateScenario(map, start2d, goal2d, obstacles, safetyRadius)
%VALIDATESCENARIO Проверяет корректность карты, точек и препятствий.

    validateattributes(map.width, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    validateattributes(map.height, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    validateattributes(map.resolution, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    validateattributes(start2d, {'numeric'}, {'real', 'finite', 'size', [1, 2]});
    validateattributes(goal2d, {'numeric'}, {'real', 'finite', 'size', [1, 2]});
    validateattributes(safetyRadius, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(obstacles, {'numeric'}, {'2d', 'real', 'finite'});

    if ~geometry.isPointInsideMap(start2d, map)
        error('Начальная точка находится вне рабочей области.');
    end

    if ~geometry.isPointInsideMap(goal2d, map)
        error('Конечная точка находится вне рабочей области.');
    end

    if ~isempty(obstacles)
        if size(obstacles, 2) ~= 4
            error('Матрица препятствий должна иметь формат [x, y, ширина, высота].');
        end

        if any(obstacles(:, 3) <= 0) || any(obstacles(:, 4) <= 0)
            error('Ширина и высота препятствий должны быть положительными.');
        end
        if any(obstacles(:, 1:2) < 0, 'all') || ...
                any(obstacles(:, 1) + obstacles(:, 3) > map.width) || ...
                any(obstacles(:, 2) + obstacles(:, 4) > map.height)
            error('geometry:ObstacleOutsideMap', 'Препятствия должны целиком находиться внутри карты.');
        end

        obstacleArea = rectangleUnionArea(obstacles);
        mapArea = map.width * map.height;

        if obstacleArea > 0.3 * mapArea
            error('geometry:ExcessObstacleArea', 'Площадь препятствий превышает %.0f%% площади рабочей области.', 30);
        end
    end

    if geometry.isPointInObstacles(start2d, obstacles, safetyRadius)
        error('Начальная точка находится внутри препятствия или зоны безопасности.');
    end

    if geometry.isPointInObstacles(goal2d, obstacles, safetyRadius)
        error('Конечная точка находится внутри препятствия или зоны безопасности.');
    end
end

function area = rectangleUnionArea(rectangles)
%RECTANGLEUNIONAREA Считает площадь объединения без повторного учёта пересечений.
% Вертикальные полосы разбиваются по всем границам прямоугольников.
    boundaries = unique([rectangles(:, 1); rectangles(:, 1) + rectangles(:, 3)]);
    area = 0;
    for strip = 1:numel(boundaries)-1
        active = rectangles(:, 1) < boundaries(strip+1) & ...
            rectangles(:, 1) + rectangles(:, 3) > boundaries(strip);
        intervals = sortrows([rectangles(active, 2), ...
            rectangles(active, 2) + rectangles(active, 4)]);
        height = 0;
        upper = -Inf;
        for index = 1:size(intervals, 1)
            height = height + max(0, intervals(index, 2) - max(upper, intervals(index, 1)));
            upper = max(upper, intervals(index, 2));
        end
        area = area + diff(boundaries(strip:strip+1))*height;
    end
end
