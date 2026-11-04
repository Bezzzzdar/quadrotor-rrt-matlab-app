function scenes = generateScenes(baseScenario, options)
%GENERATESCENES Создаёт воспроизводимые случайные конфигурации препятствий.
% Число препятствий равномерно выбирается из целочисленного диапазона countRange.
% Кандидаты размеров и положений равномерны на сетке; далее применяется отбраковка.
% Проверяются границы, непересечение прямоугольников, площадь и свободные концы.
% Связность свободной области и успех планировщиков не используются при отборе.

    validateattributes(options.sceneCount, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
    validateattributes(options.sceneSeedBase, {'numeric'}, ...
        {'scalar', 'integer', 'nonnegative', '<=', 2^32-options.sceneCount});
    settings = options.obstacles;
    validateattributes(settings.countRange, {'numeric'}, ...
        {'real', 'finite', 'integer', 'nonnegative', 'size', [1, 2]});
    validateattributes(settings.minSize_m, {'numeric'}, {'real', 'finite', 'positive', 'size', [1, 2]});
    validateattributes(settings.maxSize_m, {'numeric'}, {'real', 'finite', 'positive', 'size', [1, 2]});
    validateattributes(settings.maxAreaFraction, {'numeric'}, {'scalar', 'real', 'finite', 'positive', '<=', 0.3});
    validateattributes(settings.maxPlacementAttempts, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
    map = baseScenario.map;
    geometry.validateScenario(map, baseScenario.start2d, baseScenario.goal2d, ...
        zeros(0, 4), baseScenario.safetyRadius);
    lowerSize = ceil(settings.minSize_m / map.resolution);
    upperSize = floor(settings.maxSize_m / map.resolution);
    areaLimit = settings.maxAreaFraction * map.width * map.height;
    minimumArea = prod(lowerSize * map.resolution);
    if settings.countRange(1) > settings.countRange(2) || any(lowerSize > upperSize) || ...
            any(upperSize * map.resolution > [map.width, map.height]) || ...
            settings.countRange(2) * minimumArea > areaLimit
        error('analysis:InvalidObstacleSettings', ...
            'Размеры или число препятствий несовместимы с сеткой, картой или ограничением площади.');
    end
    template = struct('id', 0, 'seed', 0, 'obstacles', zeros(0, 4), ...
        'areaFraction', 0, 'placementAttempts', 0);
    scenes = repmat(template, options.sceneCount, 1);
    for sceneIndex = 1:options.sceneCount
        seed = options.sceneSeedBase + sceneIndex - 1;
        stream = RandStream('mt19937ar', 'Seed', seed);
        targetCount = settings.countRange(1);
        if settings.countRange(1) < settings.countRange(2)
            targetCount = randi(stream, settings.countRange);
        end
        obstacles = zeros(targetCount, 4);
        count = 0;
        area = 0;
        attempts = 0;
        while count < targetCount && attempts < settings.maxPlacementAttempts
            attempts = attempts + 1;
            sizeUnits = [randi(stream, [lowerSize(1), upperSize(1)]), ...
                randi(stream, [lowerSize(2), upperSize(2)])];
            dimensions = sizeUnits * map.resolution;
            % Резервируем минимальную площадь для ещё не размещённых препятствий.
            remainingArea = (targetCount - count - 1) * minimumArea;
            if area + prod(dimensions) + remainingArea > areaLimit, continue; end
            maximumPosition = floor(([map.width, map.height] - dimensions) / map.resolution);
            position = [randi(stream, [0, maximumPosition(1)]), ...
                randi(stream, [0, maximumPosition(2)])] * map.resolution;
            candidate = [position, dimensions];
            if geometry.isPointInObstacles(baseScenario.start2d, candidate, baseScenario.safetyRadius) || ...
                    geometry.isPointInObstacles(baseScenario.goal2d, candidate, baseScenario.safetyRadius)
                continue;
            end
            previous = obstacles(1:count, :);
            intersects = all(position < previous(:, 1:2) + previous(:, 3:4) & ...
                position + dimensions > previous(:, 1:2), 2);
            if any(intersects), continue; end
            count = count + 1;
            obstacles(count, :) = candidate;
            area = area + prod(dimensions);
        end
        if count < targetCount
            error('analysis:SceneGenerationFailed', ...
                'Для карты %d (seed=%d) размещено %d из %d препятствий за %d попыток.', ...
                sceneIndex, seed, count, targetCount, attempts);
        end
        geometry.validateScenario(map, baseScenario.start2d, baseScenario.goal2d, ...
            obstacles, baseScenario.safetyRadius);
        scenes(sceneIndex) = struct('id', sceneIndex, 'seed', seed, 'obstacles', obstacles, ...
            'areaFraction', area / (map.width * map.height), 'placementAttempts', attempts);
    end
end
