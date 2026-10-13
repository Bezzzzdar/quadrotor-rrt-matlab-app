function values = realPolynomialRoots(coefficients, lower, upper)
%REALPOLYNOMIALROOTS Выделяет вещественные корни полинома на отрезке.
% Интервалы монотонности разделяются корнями производной; на них применяется fzero.
% Литература: https://escholarship.org/uc/item/1p74v16x (раздел 6).

    first = find(coefficients ~= 0, 1);
    if isempty(first)
        values = [];
        return;
    end
    coefficients = coefficients(first:end);
    coefficients = coefficients / max(abs(coefficients));
    if isscalar(coefficients)
        values = [];
        return;
    end
    critical = realPolynomialRoots(polyder(coefficients), lower, upper);
    points = unique([lower, critical(:)', upper]);
    evaluated = polyval(coefficients, points);
    values = [];
    powers = numel(coefficients) - 1:-1:0;
    for index = 1:numel(points)
        bound = sum(abs(coefficients) .* abs(points(index)).^powers);
        if abs(evaluated(index)) <= 256 * eps * bound
            values(end + 1) = points(index); %#ok<AGROW>
        end
    end
    for index = 1:numel(points) - 1
        if sign(evaluated(index)) * sign(evaluated(index + 1)) < 0
            values(end + 1) = fzero(@(x) polyval(coefficients, x), ...
                points(index:index + 1), optimset('TolX', 1e-12)); %#ok<AGROW>
        end
    end
    values = sort(values);
    if ~isempty(values)
        values = values([true, diff(values) > 1e-9]);
    end
end
