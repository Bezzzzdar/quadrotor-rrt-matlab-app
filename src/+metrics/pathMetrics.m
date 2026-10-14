function m = pathMetrics(path, denseStep, trajectory)
%PATHMETRICS Возвращает длину и кривизну геометрического пути.
% При передаче trajectory кривизна вычисляется по полиному всей траектории.
% Литература: https://arxiv.org/pdf/math/0606007
% https://escholarship.org/uc/item/1p74v16x (раздел 6).

    if isempty(path)
        m.pathLength = NaN;
        m.maxCurvature = NaN;
        m.meanCurvature = NaN;
        m.densePath = [];
        return;
    end

    m.densePath = metrics.interpolatePath(path, denseStep);
    if nargin >= 3
        segments = prepareCurvature(trajectory);
        [m.meanCurvature, m.curvatureMeanInfo] = meanCurvatureFromSegments(segments);
        [m.maxCurvature, m.curvatureMaxInfo] = maxCurvatureFromSegments(segments);
        m.pathLength = m.curvatureMeanInfo.pathLength;
        return;
    end
    m.pathLength = metrics.calculatePathLength(m.densePath);
    curvature = metrics.calculateCurvature(m.densePath);

    if isempty(curvature)
        m.maxCurvature = 0;
        m.meanCurvature = 0;
    else
        m.maxCurvature = max(curvature);
        m.meanCurvature = mean(curvature);
    end
end
