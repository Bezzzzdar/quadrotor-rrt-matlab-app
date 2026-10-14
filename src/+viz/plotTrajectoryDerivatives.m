function plotTrajectoryDerivatives(log, axesList)
%PLOTTRAJECTORYDERIVATIVES Рисует четыре производные по x и y на заданных осях.
% Без второго аргумента создаётся отдельное окно для прежних вызовов.
    if nargin < 2
        if isempty(log)
            return;
        end
        fig = figure('Name', 'Производные фактической траектории ЛА');
        layout = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        axesList = gobjects(4, 1);
        for order = 1:4
            axesList(order) = nexttile(layout);
        end
    end
    axesList = axesList(:);
    if numel(axesList) ~= 4 || ~all(isgraphics(axesList, 'axes'))
        error('viz:InvalidDerivativeAxes', 'Требуются четыре действующие оси графиков.');
    end

    derivatives = {};
    if ~isempty(log)
        derivatives = {log.firstDerivative, log.secondDerivative, ...
            log.thirdDerivative, log.fourthDerivative};
    end
    yLabels = {'v, м/с', 'a, м/с^2', 'j, м/с^3', 's, м/с^4'};
    for order = 1:4
        ax = axesList(order);
        cla(ax);
        legend(ax, 'off');
        if ~isempty(log)
            tag = ax.Tag;
            curves = plot(ax, log.t, derivatives{order}(1:2, :)', 'LineWidth', 1.3);
            ax.Tag = tag;
            set(curves(1), 'DisplayName', 'x');
            set(curves(2), 'DisplayName', 'y');
            legend(ax, 'Location', 'northeast', 'FontSize', 10);
            xlim(ax, [log.t(1), log.t(end)]);
        end
        grid(ax, 'on');
        xlabel(ax, 'Время, с');
        ylabel(ax, yLabels{order});
        title(ax, sprintf('%d-я производная', order));
        ax.FontName = 'Times';
        ax.FontSize = 11;
    end
end
