function curvature = evaluateCurvature(segment, parameter)
%EVALUATECURVATURE Вычисляет кривизну подготовленного сегмента.
% Источник формулы: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
    velocity = zeros(3, numel(parameter));
    crossProduct = velocity;
    for component = 1:3
        velocity(component, :) = polyval(segment.velocity(component, :), parameter(:)');
        crossProduct(component, :) = polyval(segment.cross(component, :), parameter(:)');
    end
    normVelocity = sqrt(sum(velocity.^2, 1));
    normCross = sqrt(sum(crossProduct.^2, 1));
    factor = abs(polyval(segment.factor, parameter(:)'));
    curvature = reshape(normCross ./ (factor .* normVelocity.^3), size(parameter));
end
