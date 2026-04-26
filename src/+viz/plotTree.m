function plotTree(ax, tree)
%PLOTTREE Отображает дерево RRT.

if isempty(tree) || ~isfield(tree, 'nodes') || size(tree.nodes, 1) < 2
    return;
end

nodes = tree.nodes;
parents = tree.parents;

for i = 2:size(nodes, 1)
    parentIndex = parents(i);
    if parentIndex == 0
        continue;
    end

    p1 = nodes(parentIndex, :);
    p2 = nodes(i, :);
    plot(ax, [p1(1), p2(1)], [p1(2), p2(2)], ...
        'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
end
end
