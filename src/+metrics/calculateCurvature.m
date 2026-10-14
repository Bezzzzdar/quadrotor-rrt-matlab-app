function curvature = calculateCurvature(path)
%CALCULATECURVATURE Расчитывает кривизну по трём соседним точкам.

    curvature = [];
    if size(path, 1) < 3
        return;
    end

    incoming = path(2:end-1, :) - path(1:end-2, :);
    outgoing = path(3:end, :) - path(2:end-1, :);
    chord = path(3:end, :) - path(1:end-2, :);
    a = sqrt(sum(incoming.^2, 2));
    b = sqrt(sum(outgoing.^2, 2));
    c = sqrt(sum(chord.^2, 2));
    valid = a >= eps & b >= eps & c >= eps;
    if ~any(valid)
        return;
    end
    area = abs(incoming(:, 1) .* chord(:, 2) - incoming(:, 2) .* chord(:, 1)) / 2;
    curvature = 4 * area(valid) ./ (a(valid) .* b(valid) .* c(valid));
    curvature(area(valid) < eps) = 0;
end
