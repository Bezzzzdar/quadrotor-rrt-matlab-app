function segments = prepareCurvature(trajectory)
%PREPARECURVATURE Подготавливает полиномы для геометрической кривизны.
% При v=f*w общий множитель f сокращается в интеграле kappa*|v|.
% Источники: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
% https://arxiv.org/pdf/math/0606007 (кривизна и повороты касательной).

    if isfield(trajectory, 'pp')
        trajectory = trajectory.pp;
    end
    if ~isstruct(trajectory) || ~isfield(trajectory, 'form') || ~strcmp(trajectory.form, 'pp')
        error('metrics:InvalidTrajectory', 'Требуется структура траектории с полем pp или полином формы pp.');
    end
    [breaks, coefficients, pieces, order, dimension] = unmkpp(trajectory);
    if ~isscalar(dimension) || ~ismember(dimension, [2, 3]) || ...
            any(~isfinite(coefficients(:))) || any(~isfinite(breaks)) || any(diff(breaks) <= 0)
        error('metrics:InvalidTrajectory', 'Траектория должна быть конечной непрерывной кривой в двух или трёх измерениях.');
    end
    segments = repmat(struct('factor', 1, 'velocity', [], 'cross', [], ...
        'roots', [], 'multiplicities', [], 'stationary', false, ...
        'startTime', 0, 'endTime', 0), pieces, 1);
    previousEndPoint = [];
    for index = 1:pieces
        rows = (index - 1) * dimension + (1:dimension);
        position = coefficients(rows, :) .* diff(breaks(index:index + 1)).^(order - 1:-1:0);
        if dimension == 2
            position(3, :) = 0;
        end
        segments(index).startTime = breaks(index);
        segments(index).endTime = breaks(index + 1);
        if index > 1
            tolerance = 1e-8 * max(1, max(abs([previousEndPoint; position(:, end)])));
            if norm(previousEndPoint - position(:, end)) > tolerance
                error('metrics:DiscontinuousTrajectory', 'Полиномиальные сегменты не соединены по положению.');
            end
        end
        previousEndPoint = sum(position, 2);
        if order == 1
            velocity = zeros(3, 1);
        else
            velocity = position(:, 1:end - 1) .* (order - 1:-1:1);
        end
        % Отсекается только шум коэффициентов: например, производная постоянной высоты.
        scale = max(sum(abs(velocity), 2));
        negligible = sum(abs(velocity), 2) <= 256 * eps * scale;
        velocity(negligible, :) = 0;
        if ~any(velocity(:))
            segments(index).stationary = true;
            continue;
        end
        factor = 1;
        rootList = [];
        multiplicities = [];
        for root = [0, 1]
            [velocity, factor, count] = removeRoot(velocity, factor, root);
            if count > 0
                rootList(end + 1) = root; %#ok<AGROW>
                multiplicities(end + 1) = count; %#ok<AGROW>
            end
        end
        [~, reference] = max(sum(abs(velocity), 2));
        candidates = realPolynomialRoots(velocity(reference, :), 0, 1);
        for root = candidates(candidates > 1e-9 & candidates < 1 - 1e-9)
            [velocity, factor, count] = removeRoot(velocity, factor, root);
            if count > 0
                rootList(end + 1) = root; %#ok<AGROW>
                multiplicities(end + 1) = count; %#ok<AGROW>
            end
        end
        derivative = zeros(3, max(1, size(velocity, 2) - 1));
        if size(velocity, 2) > 1
            derivative = velocity(:, 1:end - 1) .* (size(velocity, 2) - 1:-1:1);
        end
        crossProduct = zeros(3, size(velocity, 2) + size(derivative, 2) - 1);
        for component = 1:3
            a = mod(component, 3) + 1;
            b = mod(component + 1, 3) + 1;
            first = conv(velocity(a, :), derivative(b, :));
            second = conv(velocity(b, :), derivative(a, :));
            difference = first - second;
            difference(abs(difference) <= 64 * eps * (abs(first) + abs(second))) = 0;
            crossProduct(component, :) = difference;
        end
        % У прямой все коэффициенты скорости сонаправлены; SVD отделяет
        % погрешность их округления от действительного изменения касательной.
        singularValues = svd(velocity);
        if numel(singularValues) < 2 || singularValues(2) <= 256 * eps * singularValues(1)
            crossProduct(:) = 0;
        end
        segments(index).factor = factor;
        segments(index).velocity = velocity;
        segments(index).cross = crossProduct;
        segments(index).roots = rootList;
        segments(index).multiplicities = multiplicities;
    end
end

function [velocity, factor, count] = removeRoot(velocity, factor, root)
%REMOVEROOT Делит все компоненты скорости на общий линейный множитель.
% Формула сокращения: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
    count = 0;
    powers = size(velocity, 2) - 1:-1:0;
    % При повторном делении сохраняется оценка ошибки исходных коэффициентов.
    bound = sum(abs(velocity) .* abs(root).^powers, 2);
    while size(velocity, 2) > 1
        values = zeros(3, 1);
        for component = 1:3
            values(component) = polyval(velocity(component, :), root);
        end
        if any(abs(values) > 1024 * eps * bound)
            break;
        end
        quotient = velocity(:, 1:end - 1);
        for column = 2:size(quotient, 2)
            quotient(:, column) = velocity(:, column) + root * quotient(:, column - 1);
        end
        velocity = quotient;
        factor = conv(factor, [1, -root]);
        count = count + 1;
    end
end
