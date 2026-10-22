function m = pathMetrics(trajectory)
%PATHMETRICS Считает длину и кривизну непрерывной полиномиальной траектории.
% Полином подготавливается один раз для обеих метрик кривизны.
% Литература: https://arxiv.org/pdf/math/0606007
% https://escholarship.org/uc/item/1p74v16x (раздел 6).

    m = struct('pathLength', NaN, 'maxCurvature', NaN, 'meanCurvature', NaN);
    if isempty(trajectory), return; end
    segments = prepareCurvature(trajectory);
    [m.meanCurvature, m.curvatureMeanInfo] = meanCurvatureFromSegments(segments);
    [m.maxCurvature, m.curvatureMaxInfo] = maxCurvatureFromSegments(segments);
    m.pathLength = m.curvatureMeanInfo.pathLength;
end
