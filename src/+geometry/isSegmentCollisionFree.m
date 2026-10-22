function result = isSegmentCollisionFree(p1, p2, map, obstacles, safetyRadius)
%ISSEGMENTCOLLISIONFREE Точно проверяет пересечение с замкнутыми прямоугольниками.
% Проверяются интервалы параметра отрезка, без дискретизации по точкам.
% https://www.pbr-book.org/4ed/Shapes/Basic_Shape_Interface#RayndashBoundsIntersections

    result = false;
    if any(~isfinite([p1, p2])) || ~geometry.isPointInsideMap(p1, map) || ...
            ~geometry.isPointInsideMap(p2, map)
        return;
    end
    direction = p2 - p1;
    for i = 1:size(obstacles, 1)
        lower = obstacles(i, 1:2) - safetyRadius;
        upper = obstacles(i, 1:2) + obstacles(i, 3:4) + safetyRadius;
        enter = 0;
        leave = 1;
        for dimension = 1:2
            if direction(dimension) == 0
                if p1(dimension) < lower(dimension) || p1(dimension) > upper(dimension)
                    enter = Inf;
                    break;
                end
            else
                times = ([lower(dimension), upper(dimension)] - p1(dimension)) / direction(dimension);
                enter = max(enter, min(times));
                leave = min(leave, max(times));
            end
        end
        if enter <= leave
            return;
        end
    end
    result = true;
end
