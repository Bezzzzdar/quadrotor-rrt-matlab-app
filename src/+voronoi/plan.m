function [path, roadmap] = plan(start2d, goal2d, map, obstacles, safetyRadius, params)
%PLAN Планирует путь по графу Вороного с настраиваемой стоимостью и подключениями.
% Базовый вариант: Малые БПЛА. Теория и практика (2015), алгоритм 9, с. 217-222.
% p<>1 и K<>3 - исследуемые расширения базовой реализации, а не формула книги.

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
    if isfield(params, 'clearanceWeight')
        validateattributes(params.clearanceWeight, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
        k2 = k1 * params.clearanceWeight;
    end
    clearanceExponent = 1;
    connectionCount = 3;
    if isfield(params, 'clearanceExponent'), clearanceExponent = params.clearanceExponent; end
    if isfield(params, 'connectionCount'), connectionCount = params.connectionCount; end
    validateattributes(clearanceExponent, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    validateattributes(connectionCount, {'numeric'}, {'scalar', 'real', 'finite', 'integer', 'positive'});
    validateattributes(k1, {'numeric'}, {'scalar', 'finite', 'positive'});
    validateattributes(k2, {'numeric'}, {'scalar', 'finite', 'positive'});

    [nodes, edges, sites] = voronoi.buildRoadmap(map, obstacles, safetyRadius);
    roadmap = struct('nodes', nodes, 'edges', edges, 'sites', sites, ...
        'weights', [], 'edgeLengths', [], 'edgeClearances', [], ...
        'startConnection', [], 'goalConnection', [], 'pathNodeIds', [], ...
        'pathCost', Inf, 'k1', k1, 'k2', k2, 'clearanceWeight', k2/k1, ...
        'clearanceExponent', clearanceExponent, 'connectionCount', connectionCount, ...
        'message', 'Допустимый маршрут Вороного не найден.');
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

    % При K=3 подключения соответствуют строкам 2-4 алгоритма 9 из книги.
    startId = size(nodes, 1) + 1;
    goalId = startId + 1;
    startEdges = connectEndpoint(start2d, startId, nodes, map, obstacles, safetyRadius, connectionCount);
    goalEdges = connectEndpoint(goal2d, goalId, nodes, map, obstacles, safetyRadius, connectionCount);
    nodes = [nodes; start2d; goal2d];
    edges = [edges; startEdges; goalEdges];
    roadmap.nodes = nodes;
    roadmap.edges = edges;
    roadmap.startConnection = startEdges;
    roadmap.goalConnection = goalEdges;
    if isempty(startEdges) || isempty(goalEdges)
        roadmap.message = sprintf('Нет безопасного подключения к %d ближайшим вершинам Вороного.', connectionCount);
        return;
    end

    [weights, lengths, clearances] = voronoi.edgeCosts(nodes, edges, sites, k1, k2, clearanceExponent);
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

function edges = connectEndpoint(point, pointId, nodes, map, obstacles, safetyRadius, connectionCount)
%CONNECTENDPOINT Проверяет K ближайших вершин без полной сортировки.
    [~, order] = mink(sum((nodes - point).^2, 2), min(connectionCount, size(nodes, 1)));
    edges = zeros(numel(order), 2);
    count = 0;
    for index = order'
        if voronoi.isSegmentFree(point, nodes(index, :), map, obstacles, safetyRadius)
            count = count + 1;
            edges(count, :) = [pointId, index];
        end
    end
    edges = edges(1:count, :);
end
