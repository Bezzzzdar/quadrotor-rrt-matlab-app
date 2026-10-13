%RUN_DEMO Пример запуска без графического интерфейса.

clear;
clc;
close all;
rng(121314)

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));

scenario = config.defaultScenario();

rrtResult = runPlannerExperiment(scenario, 'rrt');
astarResult = runPlannerExperiment(scenario, 'astar');
voronoiResult = runPlannerExperiment(scenario, 'voronoi');

plotPlannerResult(scenario, rrtResult);
plotPlannerResult(scenario, astarResult);
plotPlannerResult(scenario, voronoiResult);

comparison = [
    rrtResult.summary;
    astarResult.summary;
    voronoiResult.summary
];
comparison = addvars( ...
    comparison, ...
    ["RRT"; "A*"; "GVD Voronoi"], ...
    'Before', 1, ...
    'NewVariableNames', 'Planner');

disp('======================================================');
disp('Сравнение методов планирования');
disp(comparison);

if ~isempty(rrtResult.simLog)
    viz.plotTrajectoryDerivatives(rrtResult.simLog);
end

function plotPlannerResult(scenario, result)
%PLOTPLANNERRESULT Displays one planner result in a separate figure.

    figure('Name', result.displayName, 'NumberTitle', 'off');
    ax = gca;
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    grid(ax, 'minor');
    axis(ax, 'equal');
    xlim(ax, [0, scenario.map.width]);
    ylim(ax, [0, scenario.map.height]);
    xlabel(ax, 'x, м');
    ylabel(ax, 'y, м');
    title(ax, result.displayName);

    viz.drawObstacles(ax, scenario.obstacles);
    patch(ax, NaN, NaN, [0.35 0.35 0.35], ...
        'EdgeColor', 'k', ...
        'DisplayName', 'Препятствия');

    if strcmp(result.plannerName, 'rrt')
        viz.plotTree(ax, result.tree);
    elseif isfield(result.tree, 'edges') && ~isempty(result.tree.edges)
        edges = result.tree.edges;
        nodes = result.tree.nodes;
        x = [nodes(edges(:, 1), 1), nodes(edges(:, 2), 1), nan(size(edges, 1), 1)]';
        y = [nodes(edges(:, 1), 2), nodes(edges(:, 2), 2), nan(size(edges, 1), 1)]';
        plot(ax, x(:), y(:), '-', ...
            'Color', [0.72 0.72 0.72], ...
            'LineWidth', 0.5, ...
            'DisplayName', 'Граф Вороного');
    end

    plot(ax, scenario.start2d(1), scenario.start2d(2), ...
        'go', 'MarkerSize', 8, 'LineWidth', 2, ...
        'DisplayName', 'Начальное положение');
    plot(ax, scenario.goal2d(1), scenario.goal2d(2), ...
        'ro', 'MarkerSize', 8, 'LineWidth', 2, ...
        'DisplayName', 'Терминальное положение');

    if ~isempty(result.rawPath)
        plot(ax, result.rawPath(:, 1), result.rawPath(:, 2), ...
            'r--', 'LineWidth', 1.4, ...
            'DisplayName', 'Геометрический маршрут');
    end

    if ~isempty(result.rrtPath)
        plot(ax, result.rrtPath(:, 1), result.rrtPath(:, 2), ...
            'Color', '#FFA500', 'LineWidth', 1.6, ...
            'DisplayName', 'Маршрут после сокращений');
    end

    if ~isempty(result.path)
        plot(ax, result.path(:, 1), result.path(:, 2), ...
            'b-', 'LineWidth', 2.2, ...
            'DisplayName', 'Minimum-snap траектория');
    end

    if ~isempty(result.simLog)
        plot(ax, result.simLog.x(1, :), result.simLog.x(2, :), ...
            'm--', 'LineWidth', 1.3, ...
            'DisplayName', 'Реальная траектория движения ЛА');
    end

    legend(ax, 'Location', 'northeastoutside');
    set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times');
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', 14);
    set(findall(gcf, '-property', 'Interpreter'), 'Interpreter', 'tex');
    hold(ax, 'off');
end
