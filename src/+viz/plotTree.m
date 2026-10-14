function plotTree(ax, tree)
%PLOTTREE Отображает дерево RRT.

    if isempty(tree) || ~isfield(tree, 'nodes') || size(tree.nodes, 1) < 2
        return;
    end

    nodes = tree.nodes;
    parents = tree.parents;

    children = find(parents(:) > 0);
    if isempty(children)
        return;
    end
    ancestors = parents(children);
    % NaN разделяет рёбра в одном графическом объекте.
    x = [nodes(ancestors, 1), nodes(children, 1), nan(numel(children), 1)]';
    y = [nodes(ancestors, 2), nodes(children, 2), nan(numel(children), 1)]';
    plot(ax, x(:), y(:), 'Color', 'black', 'LineWidth', 0.5, ...
        'DisplayName', 'RRT-дерево');
end
