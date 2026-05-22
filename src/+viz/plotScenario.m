function plotScenario(ax, map, obstacles, start2d, goal2d, tree, path, simLog, rowPath, smoothPath)
%PLOTSCENARIO Отображает карту, препятствия, дерево, маршрут и траекторию.

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    grid(ax, 'minor');
    axis(ax, 'equal');
    xlim(ax, [0, map.width]);
    ylim(ax, [0, map.height]);
    xlabel(ax, 'x, м');
    ylabel(ax, 'z, м');

    viz.drawObstacles(ax, obstacles);

    legendPatch = patch(ax, NaN, NaN, [0.35 0.35 0.35], ...
                        'EdgeColor', 'k', ...
                        'DisplayName', 'Препятствия');%#ok<NASGU>

    if ~isempty(tree)
        viz.plotTree(ax, tree);
    end

    plot(ax, start2d(1), start2d(2), 'go', 'MarkerSize', 8, 'LineWidth', 2, "DisplayName", "Начальное положение");
    plot(ax, goal2d(1), goal2d(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2, "DisplayName", "Терминальное положение");

    if ~isempty(rowPath)
        plot(ax, rowPath(:, 1), rowPath(:, 2), 'r--', 'LineWidth', 2, "DisplayName", "Восстановленный RTT-маршрут");
    end

    if ~isempty(smoothPath)
        plot(ax, smoothPath(:, 1), smoothPath(:, 2), 'Color', '#FFA500', 'LineStyle', '-', 'LineWidth', 1.5, "DisplayName", "Сглаженный RTT-маршрут (Метод случайных сокращений)");
    end

    if ~isempty(path)
        plot(ax, path(:, 1), path(:, 2), 'b-', 'LineWidth', 2.5, "DisplayName", "Сглаженный маршрут (Метод минимизации четвёртой производной)");
    end

    if ~isempty(simLog)
        plot(ax, simLog.x(1, :), simLog.x(2, :), 'm--', 'LineWidth', 1.5, "DisplayName", "Реальная траектория движения ЛА");
    end

    legend(ax, "Location", "northeastoutside")
    set(findall(gcf,'-property','FontName'), 'FontName', 'Times')
    set(findall(gcf,'-property','FontSize'), 'FontSize', 14)
    set(findall(gcf,'-property','Interpreter'), 'Interpreter', 'latex')
    hold(ax, 'off');
end
