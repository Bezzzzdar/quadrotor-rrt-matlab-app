function [meanCurvature, info] = calculateMeanCurvature(trajectory)
%CALCULATEMEANCURVATURE Считает среднюю абсолютную кривизну по длине дуги.
% Вход: referenceTrajectory с полем pp либо векторный полином формы pp.
% Для гладкой кривой: meanCurvature = integral(kappa*|v| dt)/integral(|v| dt).
% При скачке касательной к полному повороту добавляется угол скачка.
% Для неподвижной траектории возвращается NaN: деление на нулевую длину.
% Литература: J. M. Sullivan, Curves of Finite Total Curvature, разделы 2-3.
% https://arxiv.org/pdf/math/0606007
% Формула кривизны: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html

    segments = prepareCurvature(trajectory);
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
        totalCurvature = totalCurvature + integral(@(u) integrand(segment, u, true), 0, 1, ...
            'Waypoints', points, 'RelTol', 1e-8, 'AbsTol', 1e-10);
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
%INTEGRAND Выбирает устойчивую плотность длины либо полного поворота.
% Литература: https://arxiv.org/pdf/math/0606007
    [~, speed, turn] = evaluateCurvature(segment, parameter);
    if turning
        value = turn;
    else
        value = speed;
    end
end
