function [maxCurvature, info] = calculateMaxCurvature(trajectory)
%CALCULATEMAXCURVATURE Ищет супремум кривизны всех полиномиальных сегментов.
% Вход: referenceTrajectory с полем pp либо векторный полином формы pp.
% Проверяются корни производной kappa^2, границы и пределы при нулевой скорости.
% Неограниченная кривизна или угол между касательными дают Inf; нулевая длина - NaN.
% Литература: R. T. Farouki et al., Path planning with Pythagorean-hodograph
% curves for unmanned or autonomous vehicles (2018), раздел 6.
% https://escholarship.org/uc/item/1p74v16x
% Здесь используется принцип поиска экстремумов, а не специальные формулы PH-кривых.
% Формула кривизны: https://www.math.hkust.edu.hk/~mafong/math4223/p_1_1_04.html

    segments = prepareCurvature(trajectory);
    [maxCurvature, info] = maxCurvatureFromSegments(segments);
end
