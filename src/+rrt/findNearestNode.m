function nearestIndex = findNearestNode(nodes, point)
%FINDNEARESTNODE Ищет ближайшую вершину дерева.

    diff = nodes - point;
    distSquared = diff(:, 1).^2 + diff(:, 2).^2;
    [~, nearestIndex] = min(distSquared);
end
