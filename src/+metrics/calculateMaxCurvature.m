function [maxCurvature, info] = calculateMaxCurvature(trajectory)
%CALCULATEMAXCURVATURE Ищет супремум кривизны всех полиномиальных сегментов.
% Вход: referenceTrajectory с полем pp либо векторный полином формы pp.
% Проверяются корни производной kappa^2, границы и пределы при нулевой скорости.
% Неограниченная кривизна или угол между касательными дают Inf; нулевая длина — NaN.
% Литература: R. T. Farouki et al., Path planning with Pythagorean-hodograph
% curves for unmanned or autonomous vehicles (2018), раздел 6.
% https://escholarship.org/uc/item/1p74v16x
% Здесь используется принцип поиска экстремумов, а не специальные формулы PH-кривых.
% Формула кривизны: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html

    segments = prepareCurvature(trajectory);
    maxCurvature = NaN;
    info = struct('time', NaN, 'unbounded', false, 'stationaryTimes', []);
    [~, corners] = trajectoryCorners(segments);
    if ~isempty(corners)
        maxCurvature = Inf;
        info.time = corners(1);
        info.unbounded = true;
        return;
    end
    for index = 1:numel(segments)
        segment = segments(index);
        if segment.stationary
            continue;
        end
        points = [0, 1];
        for root = segment.roots
            time = segment.startTime + root * (segment.endTime - segment.startTime);
            info.stationaryTimes(end + 1) = time;
            value = curvatureLimit(segment, root);
            [maxCurvature, info] = updateMaximum(maxCurvature, info, value, time);
        end
        if isinf(maxCurvature)
            return;
        end
        squaredCross = sumSquares(segment.cross);
        squaredVelocity = sumSquares(segment.velocity);
        factor = segment.factor;
        % Производная N/(f^2*S^3) обращается в нуль при N'*f*S-2*N*f'*S-3*N*f*S'=0.
        numerator = addPolynomial( ...
            conv(conv(polyder(squaredCross), factor), squaredVelocity), ...
            -2 * conv(conv(squaredCross, polyder(factor)), squaredVelocity));
        numerator = addPolynomial(numerator, ...
            -3 * conv(conv(squaredCross, factor), polyder(squaredVelocity)));
        points = unique([points, realPolynomialRoots(numerator, 0, 1)]);
        for parameter = points
            if any(abs(segment.roots - parameter) <= 1e-9)
                continue;
            end
            value = evaluateCurvature(segment, parameter);
            time = segment.startTime + parameter * (segment.endTime - segment.startTime);
            [maxCurvature, info] = updateMaximum(maxCurvature, info, value, time);
        end
    end
end

function result = sumSquares(coefficients)
%SUMSQUARES Строит полином квадрата нормы векторного полинома.
% Формула: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html
    result = 0;
    for component = 1:size(coefficients, 1)
        result = addPolynomial(result, conv(coefficients(component, :), coefficients(component, :)));
    end
end

function result = addPolynomial(a, b)
%ADDPOLYNOMIAL Складывает полиномы после выравнивания степеней.
% Применение к экстремумам: https://escholarship.org/uc/item/1p74v16x
    count = max(numel(a), numel(b));
    result = [zeros(1, count - numel(a)), a] + [zeros(1, count - numel(b)), b];
end

function [maximum, info] = updateMaximum(maximum, info, value, time)
%UPDATEMAXIMUM Сохраняет наибольшее значение и соответствующее время.
% Принцип сравнения кандидатов: https://escholarship.org/uc/item/1p74v16x
    if isnan(maximum) || value > maximum
        maximum = value;
        info.time = time;
        info.unbounded = isinf(value);
    end
end
