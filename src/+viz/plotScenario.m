function plotScenario(ax, map, obstacles, start2d, goal2d, tree, path, simLog)
%PLOTSCENARIO Отображает карту, препятствия, дерево, маршрут и траекторию.

cla(ax);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [0, map.width]);
ylim(ax, [0, map.height]);
xlabel(ax, 'X, м');
ylabel(ax, 'Y, м');
title(ax, 'Планирование маршрута мультироторного ЛА методом RRT');

viz.drawObstacles(ax, obstacles);

if ~isempty(tree)
    viz.plotTree(ax, tree);
end

plot(ax, start2d(1), start2d(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(ax, goal2d(1), goal2d(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

if ~isempty(path)
    plot(ax, path(:, 1), path(:, 2), 'b-', 'LineWidth', 2.5);
end

if ~isempty(simLog)
    plot(ax, simLog.x(1, :), simLog.x(2, :), 'm--', 'LineWidth', 1.5);
end

hold(ax, 'off');
end
