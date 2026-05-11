function smoothPath = shortcutSmoothPath(path, map, obstacles, safetyRadius, checkStep, iterations)
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

            if geometry.isSegmentCollisionFree(p1, p2, map, obstacles, safetyRadius, checkStep)
                smoothPath = [smoothPath(1:i, :); smoothPath(j:end, :)];
            end
        end
    end

    smoothPath = insertIntermediatePoints(smoothPath, 1);
end

function refinedPath = insertIntermediatePoints(path, pointsPerSegment)
%INSERTINTERMEDIATEPOINTS Inserts a fixed number of points per segment.

    if pointsPerSegment <= 0
        refinedPath = path;
        return;
    end

    segmentCount = size(path, 1) - 1;
    rowsPerSegment = pointsPerSegment + 1;
    refinedPath = zeros(segmentCount * rowsPerSegment + 1, size(path, 2));

    outRow = 1;
    for i = 1:segmentCount
        p1 = path(i, :);
        p2 = path(i + 1, :);
        refinedPath(outRow, :) = p1;
        outRow = outRow + 1;

        for k = 1:pointsPerSegment
            t = k / (pointsPerSegment + 1);
            refinedPath(outRow, :) = p1 + t * (p2 - p1);
            outRow = outRow + 1;
        end
    end

    refinedPath(outRow, :) = path(end, :);
end
