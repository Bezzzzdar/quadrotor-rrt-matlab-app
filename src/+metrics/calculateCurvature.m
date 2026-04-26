function curvature = calculateCurvature(path)
%CALCULATECURVATURE Расчитывает кривизну по трём соседним точкам.

curvature = [];
if size(path, 1) < 3
    return;
end

for i = 2:size(path, 1) - 1
    p1 = path(i - 1, :);
    p2 = path(i, :);
    p3 = path(i + 1, :);

    a = norm(p2 - p1);
    b = norm(p3 - p2);
    c = norm(p3 - p1);

    if a < eps || b < eps || c < eps
        continue;
    end

    triangleArea = abs(0.5 * det([p2 - p1; p3 - p1]));

    if triangleArea < eps
        kappa = 0;
    else
        kappa = 4 * triangleArea / (a * b * c);
    end

    curvature = [curvature; kappa]; %#ok<AGROW>
end
end
