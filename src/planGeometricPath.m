function planned = planGeometricPath(scenario, plannerName)
%PLANGEOMETRICPATH Отделяет измеряемое планирование от обработки траектории.
% Раздельные метрики: https://www.kavrakilab.org/publications/moll-sucan2015benchmarking-motion-planning.pdf
    geometry.validateScenario(scenario.map, scenario.start2d, scenario.goal2d, ...
        scenario.obstacles, scenario.safetyRadius);
    params = scenario.rrt;
    if params.goalThreshold <= 0, params.goalThreshold = params.stepSize; end
    plannerName = lower(string(plannerName));
    startTime = tic;
    switch plannerName
        case "rrt"
            [path, tree] = rrt.plan(scenario.start2d, scenario.goal2d, scenario.map, ...
                scenario.obstacles, scenario.safetyRadius, params);
            displayName = "RRT";
        case "astar"
            astarParams = struct();
            if isfield(scenario, 'astar'), astarParams = scenario.astar; end
            [path, tree] = astar.plan(scenario.start2d, scenario.goal2d, scenario.map, ...
                scenario.obstacles, scenario.safetyRadius, astarParams);
            displayName = "A*";
        case "voronoi"
            voronoiParams = struct();
            if isfield(scenario, 'voronoi'), voronoiParams = scenario.voronoi; end
            [path, tree] = voronoi.plan(scenario.start2d, scenario.goal2d, scenario.map, ...
                scenario.obstacles, scenario.safetyRadius, voronoiParams);
            displayName = "GVD Voronoi";
        otherwise
            error('planning:UnknownPlanner', 'Неизвестный планировщик "%s".', plannerName);
    end
    planned = struct('plannerName', char(plannerName), 'displayName', char(displayName), ...
        'path', path, 'tree', tree, 'calculationTime', toc(startTime));
end
