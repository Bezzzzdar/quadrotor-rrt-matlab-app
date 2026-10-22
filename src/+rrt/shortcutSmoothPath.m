function smoothPath = shortcutSmoothPath(path, map, obstacles, safetyRadius, iterations)
%SHORTCUTSMOOTHPATH Сглаживает маршрут методом случайных сокращений.

    smoothPath = path;

    if size(smoothPath, 1) <= 1
        return;
    end

    if size(smoothPath, 1) > 2
        for k = 1:iterations
            n = size(smoothPath, 1);

            if n <= 2
                break;
            end

            i = randi([1, n - 1]);
            j = randi([i + 1, n]);

            if j <= i + 1
                continue;
            end

            p1 = smoothPath(i, :);
            p2 = smoothPath(j, :);

            if geometry.isSegmentCollisionFree(p1, p2, map, obstacles, safetyRadius)
                smoothPath = [smoothPath(1:i, :); smoothPath(j:end, :)];
            end
        end
    end

    smoothPath = insertIntermediatePoints(smoothPath);
end

function refinedPath = insertIntermediatePoints(path)
%INSERTINTERMEDIATEPOINTS Добавляет середину каждого сегмента пути.
    segmentCount = size(path, 1) - 1;
    refinedPath = zeros(2 * segmentCount + 1, size(path, 2));
    refinedPath(1:2:end, :) = path;
    refinedPath(2:2:end, :) = path(1:end-1, :) + 0.5 * diff(path, 1, 1);
end
