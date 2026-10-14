function [free, detail] = isPolynomialTrajectoryFree(pp, map, obstacles, safetyRadius)
%ISPOLYNOMIALTRAJECTORYFREE Проверяет непрерывные полиномы, а не только отсчёты.
% Корни пересечений с границами разбивают участок на интервалы постоянной занятости.
% Это численная проверка с допуском, а не формальный интервальный сертификат.
% Представление и корни: https://www.mathworks.com/help/matlab/ref/unmkpp.html
% https://www.mathworks.com/help/matlab/ref/roots.html
    [breaks, coefficients, pieces, order, dimension] = unmkpp(pp);
    dimension = prod(dimension);
    if dimension < 2 || any(~isfinite(coefficients), 'all') || ...
            any(~isfinite(breaks)) || any(diff(breaks) <= 0)
        error('geometry:InvalidPolynomial', 'Нужен конечный полином с двумя координатами и возрастающими узлами.');
    end
    tolerance = 1e-9 * max([1, map.width, map.height]);
    detail = struct('piece', NaN, 'obstacle', NaN, 'time', NaN, 'message', '');
    free = false;
    for piece = 1:pieces
        duration = breaks(piece+1) - breaks(piece);
        rows = (piece-1)*dimension + (1:2);
        local = coefficients(rows, :) .* duration.^(order-1:-1:0);
        critical = unique([0; 1; unitRoots(polyder(local(1, :))); unitRoots(polyder(local(2, :)))]);
        positions = [polyval(local(1, :), critical), polyval(local(2, :), critical)];
        outside = positions(:, 1) < -tolerance | positions(:, 1) > map.width+tolerance | ...
            positions(:, 2) < -tolerance | positions(:, 2) > map.height+tolerance;
        if any(outside)
            detail.piece = piece;
            detail.time = breaks(piece) + duration*critical(find(outside, 1));
            detail.message = 'Траектория выходит за границу карты.';
            return;
        end
        lowerBounds = min(positions, [], 1);
        upperBounds = max(positions, [], 1);
        for obstacle = 1:size(obstacles, 1)
            lower = obstacles(obstacle, 1:2) - safetyRadius;
            upper = obstacles(obstacle, 1:2) + obstacles(obstacle, 3:4) + safetyRadius;
            if any(upperBounds < lower-tolerance | lowerBounds > upper+tolerance)
                continue;
            end
            cuts = critical;
            for coordinate = 1:2
                for boundary = [lower(coordinate), upper(coordinate)]
                    equation = local(coordinate, :);
                    equation(end) = equation(end) - boundary;
                    cuts = [cuts; unitRoots(equation)]; %#ok<AGROW>
                end
            end
            cuts = unique(cuts);
            probes = sort([cuts; (cuts(1:end-1)+cuts(2:end))/2]);
            positions = [polyval(local(1, :), probes), polyval(local(2, :), probes)];
            collision = all(positions >= lower-tolerance & positions <= upper+tolerance, 2);
            if any(collision)
                detail.piece = piece;
                detail.obstacle = obstacle;
                detail.time = breaks(piece) + duration*probes(find(collision, 1));
                detail.message = 'Траектория пересекает препятствие с учётом безопасного отступа.';
                return;
            end
        end
    end
    free = true;
end

function values = unitRoots(coefficients)
%UNITROOTS Находит вещественные корни в нормированном интервале [0, 1].
    first = find(coefficients ~= 0, 1);
    if isempty(first) || first == numel(coefficients)
        values = zeros(0, 1);
        return;
    end
    candidates = roots(coefficients(first:end));
    selected = abs(imag(candidates)) <= 1e-7*(1+abs(real(candidates))) & ...
        real(candidates) >= -1e-8 & real(candidates) <= 1+1e-8;
    values = min(1, max(0, real(candidates(selected))));
end
