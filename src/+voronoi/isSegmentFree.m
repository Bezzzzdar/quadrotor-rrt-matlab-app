function free = isSegmentFree(a, b, map, obstacles, safetyRadius)
%ISSEGMENTFREE Exact segment/closed inflated rectangle intersection test.

    free = false;
    if any(~isfinite([a, b])) || ~geometry.isPointInsideMap(a, map) || ...
            ~geometry.isPointInsideMap(b, map)
        return;
    end
    direction = b - a;
    for index = 1:size(obstacles, 1)
        lower = obstacles(index, 1:2) - safetyRadius;
        upper = obstacles(index, 1:2) + obstacles(index, 3:4) + safetyRadius;
        enter = 0;
        leave = 1;
        for dimension = 1:2
            if direction(dimension) == 0
                if a(dimension) < lower(dimension) || a(dimension) > upper(dimension)
                    enter = Inf;
                    break;
                end
            else
                times = ([lower(dimension), upper(dimension)] - a(dimension)) / direction(dimension);
                enter = max(enter, min(times));
                leave = min(leave, max(times));
            end
        end
        if enter <= leave
            return;
        end
    end
    free = true;
end
