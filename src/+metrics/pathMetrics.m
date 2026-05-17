function m = pathMetrics(path, denseStep)
%PATHMETRICS Возвращает длину и кривизну геометрического пути.

    if isempty(path)
        m.pathLength = NaN;
        m.maxCurvature = NaN;
        m.meanCurvature = NaN;
        m.densePath = [];
        return;
    end

    m.densePath = metrics.interpolatePath(path, denseStep);
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
