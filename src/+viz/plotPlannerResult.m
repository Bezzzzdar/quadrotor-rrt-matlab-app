function fig = plotPlannerResult(scenario, result)
%PLOTPLANNERRESULT Объединяет маршрут, ошибки и четыре производные в одном окне.
    screen = get(groot, 'ScreenSize');
    fig = figure('Name', result.displayName, 'NumberTitle', 'off', ...
        'Position', [screen(1) + 40, screen(2) + 40, ...
        min(1400, screen(3) - 80), min(900, screen(4) - 100)]);
    layout = tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Маршруту отведены три ряда слева; ошибки расположены под ним.
    mapAxes = nexttile(layout, 1, [3, 1]);
    mapAxes.Tag = 'DemoMapAxes';
    errorAxes = nexttile(layout, 7);
    errorAxes.Tag = 'DemoErrorAxes';
    derivativeAxes = gobjects(4, 1);
    for order = 1:4
        derivativeAxes(order) = nexttile(layout, 2 * order);
        derivativeAxes(order).Tag = sprintf('DemoDerivativeAxes%d', order);
    end

    viz.plotScenario(mapAxes, scenario.map, scenario.obstacles, ...
        scenario.start2d, scenario.goal2d, result.tree, result.path, ...
        result.simLog, result.rawPath, result.rrtPath, result.displayName);
    mapAxes.FontSize = 11;
    legend(mapAxes, 'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 9);
    viz.plotTrackingError(errorAxes, result.simLog);
    errorAxes.FontName = 'Times';
    errorAxes.FontSize = 11;
    viz.plotTrajectoryDerivatives(result.simLog, derivativeAxes);

    timeAxes = [errorAxes; derivativeAxes];
    linkaxes(timeAxes, 'x');
    if ~isempty(result.simLog)
        xlim(errorAxes, [result.simLog.t(1), result.simLog.t(end)]);
    end
end
