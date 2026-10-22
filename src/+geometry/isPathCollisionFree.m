function free = isPathCollisionFree(path, map, obstacles, safetyRadius)
%ISPATHCOLLISIONFREE Проверяет все отрезки ломаной, включая вырожденный путь.
% https://www.pbr-book.org/4ed/Shapes/Basic_Shape_Interface#RayndashBoundsIntersections
    free = ~isempty(path) && size(path, 2) == 2 && all(isfinite(path), 'all');
    if ~free, return; end
    for index = 1:max(1, size(path, 1)-1)
        next = min(index+1, size(path, 1));
        if ~geometry.isSegmentCollisionFree(path(index, :), path(next, :), map, obstacles, safetyRadius)
            free = false;
            return;
        end
    end
end
