function result = isPointInsideMap(point, map)
%ISPOINTINSIDEMAP Проверяет, находится ли точка внутри рабочей области.

    result = point(1) >= 0 && point(1) <= map.width && ...
            point(2) >= 0 && point(2) <= map.height;
end
