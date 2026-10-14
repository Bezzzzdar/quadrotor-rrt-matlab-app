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
    [meanCurvature, info] = meanCurvatureFromSegments(segments);
end
