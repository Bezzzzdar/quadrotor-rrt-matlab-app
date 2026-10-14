function [meanCurvature, info] = meanCurvatureFromSegments(segments)
%MEANCURVATUREFROMSEGMENTS Интегрирует кривизну подготовленных сегментов.
% Литература: https://arxiv.org/pdf/math/0606007 (разделы 2-3).
    lengthValue = 0;
    totalCurvature = 0;
    for index = 1:numel(segments)
        segment = segments(index);
        if segment.stationary
            continue;
        end
        points = segment.roots(segment.roots > 0 & segment.roots < 1);
        lengthValue = lengthValue + integral(@(u) integrand(segment, u, false), 0, 1, ...
            'Waypoints', points, 'RelTol', 1e-8, 'AbsTol', 1e-10);
        if any(segment.cross(:))
            totalCurvature = totalCurvature + integral(@(u) integrand(segment, u, true), 0, 1, ...
                'Waypoints', points, 'RelTol', 1e-8, 'AbsTol', 1e-10);
        end
    end
    [cornerAngle, cornerTimes] = trajectoryCorners(segments);
    totalCurvature = totalCurvature + cornerAngle;
    meanCurvature = NaN;
    if lengthValue > 0
        meanCurvature = totalCurvature / lengthValue;
    end
    info = struct('pathLength', lengthValue, 'totalCurvature', totalCurvature, ...
        'cornerTimes', cornerTimes, 'cornerAngle', cornerAngle);
end

function value = integrand(segment, parameter, turning)
%INTEGRAND Считает только требуемую плотность: длину либо полный поворот.
% Формула: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
% Интеграл поворота: https://arxiv.org/pdf/math/0606007
    velocity = zeros(3, numel(parameter));
    for component = 1:3
        velocity(component, :) = polyval(segment.velocity(component, :), parameter(:)');
    end
    squaredVelocity = sum(velocity.^2, 1);
    if turning
        crossProduct = zeros(size(velocity));
        for component = 1:3
            crossProduct(component, :) = polyval(segment.cross(component, :), parameter(:)');
        end
        value = sqrt(sum(crossProduct.^2, 1)) ./ squaredVelocity;
    else
        value = abs(polyval(segment.factor, parameter(:)')) .* sqrt(squaredVelocity);
    end
    value = reshape(value, size(parameter));
end
