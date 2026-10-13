function value = curvatureLimit(segment, root)
%CURVATURELIMIT Находит предел кривизны в точке нулевой скорости.
% Сравниваются порядки нулей числителя и знаменателя формулы кривизны.
% Источник: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
    crossProduct = segment.cross;
    if ~any(crossProduct(:))
        value = 0;
        return;
    end
    numeratorOrder = 0;
    while size(crossProduct, 2) > 1 && isZeroAt(crossProduct, root)
        quotient = crossProduct(:, 1:end - 1);
        for column = 2:size(quotient, 2)
            quotient(:, column) = crossProduct(:, column) + root * quotient(:, column - 1);
        end
        crossProduct = quotient;
        numeratorOrder = numeratorOrder + 1;
    end
    [~, index] = min(abs(segment.roots - root));
    denominatorOrder = segment.multiplicities(index);
    if numeratorOrder < denominatorOrder
        value = Inf;
    elseif numeratorOrder > denominatorOrder
        value = 0;
    else
        reduced = segment.factor;
        for power = 1:denominatorOrder
            quotient = reduced(1:end - 1);
            for column = 2:numel(quotient)
                quotient(column) = reduced(column) + root * quotient(column - 1);
            end
            reduced = quotient;
        end
        v = zeros(3, 1);
        q = v;
        for component = 1:3
            v(component) = polyval(segment.velocity(component, :), root);
            q(component) = polyval(crossProduct(component, :), root);
        end
        value = norm(q) / (abs(polyval(reduced, root)) * norm(v)^3);
    end
end

function result = isZeroAt(coefficients, root)
%ISZEROAT Проверяет нуль с учётом погрешности вычисления полинома.
% Используется в формуле: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
    powers = size(coefficients, 2) - 1:-1:0;
    bound = sum(abs(coefficients) .* abs(root).^powers, 2);
    values = zeros(3, 1);
    for component = 1:3
        values(component) = polyval(coefficients(component, :), root);
    end
    result = all(abs(values) <= 256 * eps * bound);
end
