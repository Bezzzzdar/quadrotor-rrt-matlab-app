function plotScenario(ax, map, obstacles, start2d, goal2d, tree, path, simLog, rawPath, smoothPath, plannerName)
%PLOTSCENARIO Отображает карту, препятствия, дерево, маршрут и траекторию.

    cla(ax);
    if nargin >= 11
        title(ax, plannerName);
    end
    hold(ax, 'on');
    grid(ax, 'on');
    grid(ax, 'minor');
    axis(ax, 'equal');
    xlim(ax, [0, map.width]);
    ylim(ax, [0, map.height]);
    xlabel(ax, 'x, м');
    ylabel(ax, 'y, м');

    viz.drawObstacles(ax, obstacles);

    patch(ax, NaN, NaN, [0.35 0.35 0.35], ...
                        'EdgeColor', 'k', ...
                        'DisplayName', 'Препятствия');

    if ~isempty(tree)
        if isfield(tree, 'nodes') && isfield(tree, 'edges') && ~isempty(tree.edges)
            edges = tree.edges;
            nodes = tree.nodes;
            x = [nodes(edges(:, 1), 1), nodes(edges(:, 2), 1), nan(size(edges, 1), 1)]';
            y = [nodes(edges(:, 1), 2), nodes(edges(:, 2), 2), nan(size(edges, 1), 1)]';
            plot(ax, x(:), y(:), '-', 'Color', [0.72 0.72 0.72], ...
                'LineWidth', 0.5, 'DisplayName', 'Граф Вороного');
        elseif isfield(tree, 'nodes') && isfield(tree, 'parents')
            viz.plotTree(ax, tree);
        end
    end

    plot(ax, start2d(1), start2d(2), 'go', 'MarkerSize', 8, 'LineWidth', 2, "DisplayName", "Начальное положение");
    plot(ax, goal2d(1), goal2d(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2, "DisplayName", "Терминальное положение");

    if ~isempty(rawPath)
        plot(ax, rawPath(:, 1), rawPath(:, 2), 'r--', 'LineWidth', 1.4, 'DisplayName', 'Геометрический маршрут');
    end

    if ~isempty(smoothPath)
        plot(ax, smoothPath(:, 1), smoothPath(:, 2), 'Color', '#FFA500', 'LineWidth', 1.6, 'DisplayName', 'Маршрут после сокращений');
    end

    if ~isempty(path)
        plot(ax, path(:, 1), path(:, 2), 'b-', 'LineWidth', 2.2, 'DisplayName', 'Minimum-snap траектория');
    end

    if ~isempty(simLog)
        plot(ax, simLog.x(1, :), simLog.x(2, :), 'm--', 'LineWidth', 1.5, "DisplayName", "Реальная траектория движения ЛА");
    end

    plotLegend = legend(ax, 'Location', 'northeastoutside');
    % Оформление карты не должно менять поля и другие вкладки приложения.
    set(findall(ax, '-property', 'FontName'), 'FontName', 'Times');
    set(findall(ax, '-property', 'FontSize'), 'FontSize', 14);
    set(findall(ax, '-property', 'Interpreter'), 'Interpreter', 'tex');
    set(plotLegend, 'FontName', 'Times', 'FontSize', 14, 'Interpreter', 'tex');
    hold(ax, 'off');
end
