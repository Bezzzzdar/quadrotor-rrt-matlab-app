function [path, roadmap] = plan(start2d, goal2d, map, obstacles, safetyRadius, params)
%PLAN Voronoi waypoint planning, Algorithm 9 and Eq. (12.1), pp. 217-222.
% Source: Malye Bespilotnye Letatelnye Apparaty. Teoria I Praktika (2015).
% Polygon perimeters are sampled as in Fig. 12.5; map walls bound the diagram.

    geometry.validateScenario(map, start2d, goal2d, obstacles, safetyRadius);
    if ~isscalar(map.resolution) || ~isfinite(map.resolution) || map.resolution <= 0
        error('voronoi:InvalidResolution', 'Map resolution must be finite and positive.');
    end
    if nargin < 6
        params = struct();
    end
    k1 = 0.1;
    k2 = 0.9;
    if isfield(params, 'k1'), k1 = params.k1; end
    if isfield(params, 'k2'), k2 = params.k2; end
    validateattributes(k1, {'numeric'}, {'scalar', 'finite', 'positive'});
    validateattributes(k2, {'numeric'}, {'scalar', 'finite', 'positive'});

    [nodes, edges, sites] = voronoi.buildRoadmap(map, obstacles, safetyRadius);
    roadmap = struct('nodes', nodes, 'edges', edges, 'sites', sites, ...
        'weights', [], 'edgeLengths', [], 'edgeClearances', [], ...
        'startConnection', [], 'goalConnection', [], 'pathNodeIds', [], ...
        'pathCost', Inf, 'k1', k1, 'k2', k2, 'message', 'No feasible Voronoi route.');
    path = [];
    if isequal(start2d, goal2d)
        path = start2d;
        roadmap.message = 'Start and goal coincide.';
        roadmap.pathCost = 0;
        return;
    end
    if isempty(nodes)
        roadmap.message = 'No collision-free finite Voronoi edges.';
        return;
    end

    % Add ps and pe, then the three nearest vertices (Algorithm 9, lines 2-4).
    startId = size(nodes, 1) + 1;
    goalId = startId + 1;
    startEdges = connectEndpoint(start2d, startId, nodes, map, obstacles, safetyRadius);
    goalEdges = connectEndpoint(goal2d, goalId, nodes, map, obstacles, safetyRadius);
    nodes = [nodes; start2d; goal2d];
    edges = [edges; startEdges; goalEdges];
    roadmap.nodes = nodes;
    roadmap.edges = edges;
    roadmap.startConnection = startEdges;
    roadmap.goalConnection = goalEdges;
    if isempty(startEdges) || isempty(goalEdges)
        roadmap.message = 'No feasible connection to the three nearest Voronoi vertices.';
        return;
    end

    [weights, lengths, clearances] = voronoi.edgeCosts(nodes, edges, sites, k1, k2);
    roadmap.weights = weights;
    roadmap.edgeLengths = lengths;
    roadmap.edgeClearances = clearances;
    network = graph(edges(:, 1), edges(:, 2), weights, size(nodes, 1));
    [ids, cost] = shortestpath(network, startId, goalId, 'Method', 'positive');
    roadmap.pathNodeIds = ids;
    roadmap.pathCost = cost;
    if isempty(ids)
        roadmap.message = 'Start and goal belong to disconnected Voronoi components.';
        return;
    end
    path = nodes(ids, :);
    roadmap.message = 'Route found using Voronoi edge costs and Dijkstra search.';
end

function edges = connectEndpoint(point, pointId, nodes, map, obstacles, safetyRadius)
    [~, order] = sort(sum((nodes - point).^2, 2));
    edges = zeros(0, 2);
    for index = order(1:min(3, numel(order)))'
        if voronoi.isSegmentFree(point, nodes(index, :), map, obstacles, safetyRadius)
            edges(end + 1, :) = [pointId, index]; %#ok<AGROW>
        end
    end
end
