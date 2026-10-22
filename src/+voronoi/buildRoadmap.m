function [nodes, edges, sites] = buildRoadmap(map, obstacles, safetyRadius)
%BUILDROADMAP Voronoi diagram of perimeter samples, with infeasible edges removed.

    rectangles = [obstacles; 0, 0, map.width, map.height];
    samples = cell(size(rectangles, 1), 1);
    for index = 1:size(rectangles, 1)
        rect = rectangles(index, :);
        corners = [rect(1:2); rect(1) + rect(3), rect(2); ...
            rect(1) + rect(3), rect(2) + rect(4); rect(1), rect(2) + rect(4)];
        sides = cell(4, 1);
        for side = 1:4
            a = corners(side, :);
            b = corners(mod(side, 4) + 1, :);
            count = max(1, ceil(norm(b - a) / map.resolution));
            if index == size(rectangles, 1)
                % At least 12 wall samples satisfy Algorithm 9's |Q| >= 10.
                count = max(3, count);
            end
            sides{side} = a + ((0:count - 1)' / count) .* (b - a);
        end
        samples{index} = vertcat(sides{:});
    end
    sites = unique(vertcat(samples{:}), 'rows');

    % Finite Voronoi edges are dual to adjacent Delaunay triangles.
    triangulation = delaunayTriangulation(sites);
    vertices = circumcenter(triangulation);
    adjacent = neighbors(triangulation);
    triangleIds = repmat((1:size(adjacent, 1))', 1, 3);
    finite = isfinite(adjacent) & adjacent > triangleIds;
    pairs = [triangleIds(finite), adjacent(finite)];

    % Cocircular perimeter samples can produce coincident circumcenters.
    tolerance = max(map.width, map.height) * 1e-10;
    [~, first, vertexIds] = unique(round(vertices / tolerance), 'rows');
    nodes = vertices(first, :);
    edges = sort([vertexIds(pairs(:, 1)), vertexIds(pairs(:, 2))], 2);
    edges = unique(edges(edges(:, 1) ~= edges(:, 2), :), 'rows');
    allowed = false(size(edges, 1), 1);
    for index = 1:size(edges, 1)
        allowed(index) = geometry.isSegmentCollisionFree( ...
            nodes(edges(index, 1), :), nodes(edges(index, 2), :), ...
            map, obstacles, safetyRadius);
    end
    edges = edges(allowed, :);
    used = unique(edges(:));
    ids = zeros(size(nodes, 1), 1);
    ids(used) = 1:numel(used);
    nodes = nodes(used, :);
    edges = reshape(ids(edges), size(edges));
end
