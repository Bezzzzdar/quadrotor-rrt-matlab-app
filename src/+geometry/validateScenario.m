function validateScenario(map, start2d, goal2d, obstacles, safetyRadius)
%VALIDATESCENARIO Проверяет корректность карты, точек и препятствий.

    if map.width <= 0 || map.height <= 0
        error('Размеры рабочей области должны быть положительными.');
    end

    if map.resolution <= 0
        error('Разрешение карты должно быть положительным.');
    end

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

        obstacleArea = sum(obstacles(:, 3) .* obstacles(:, 4));
        mapArea = map.width * map.height;

        if obstacleArea > 0.3 * mapArea
            error('Площадь препятствий превышает 30%% площади рабочей области.');
        end
    end

    if geometry.isPointInObstacles(start2d, obstacles, safetyRadius)
        error('Начальная точка находится внутри препятствия или зоны безопасности.');
    end

    if geometry.isPointInObstacles(goal2d, obstacles, safetyRadius)
        error('Конечная точка находится внутри препятствия или зоны безопасности.');
    end
end
