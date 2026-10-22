function plotTrajectoryDerivatives(log, axesList, components, includeReference)
%PLOTTRAJECTORYDERIVATIVES Рисует четыре производные по x и y на заданных осях.
% Один и тот же расчёт графиков используется приложением и demo.
    if nargin < 3, components = 1:2; end
    if nargin < 4, includeReference = false; end
    axesList = axesList(:);
    if numel(axesList) ~= 4 || ~all(isgraphics(axesList, 'axes'))
        error('viz:InvalidDerivativeAxes', 'Требуются четыре действующие оси графиков.');
    end

    derivatives = {};
    reference = {};
    if ~isempty(log)
        derivatives = {log.firstDerivative, log.secondDerivative, ...
            log.thirdDerivative, log.fourthDerivative};
        if includeReference, reference = {log.vd, log.ad, log.jerk, log.snap}; end
    end
    componentNames = {'x', 'y'};
    derivativeNames = {'Скорость', 'Ускорение', 'Рывок', 'Snap'};
    units = {'м/с', 'м/с^2', 'м/с^3', 'м/с^4'};
    colors = lines(2);
    yLabels = {'v, м/с', 'a, м/с^2', 'j, м/с^3', 's, м/с^4'};
    for order = 1:4
        ax = axesList(order);
        cla(ax);
        legend(ax, 'off');
        if ~isempty(log)
            hold(ax, 'on');
            for component = components
                name = componentNames{component};
                actualLabel = name;
                if includeReference, actualLabel = sprintf('%s (ЛА)', name); end
                plot(ax, log.t, derivatives{order}(component, :), ...
                    'Color', colors(component, :), 'LineWidth', 1.3, 'DisplayName', actualLabel);
                if includeReference
                    plot(ax, log.t, reference{order}(component, :), '--', ...
                        'Color', colors(component, :), 'LineWidth', 1.1, ...
                        'DisplayName', sprintf('%s (опорная)', name));
                end
            end
            if includeReference
                legend(ax, 'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 10);
            else
                legend(ax, 'Location', 'northeast', 'FontSize', 10);
            end
            xlim(ax, [log.t(1), log.t(end)]);
            hold(ax, 'off');
        end
        grid(ax, 'on');
        xlabel(ax, 'Время, с');
        if includeReference
            ylabel(ax, units{order});
            title(ax, sprintf('%d-я производная: %s', order, derivativeNames{order}));
        else
            ylabel(ax, yLabels{order});
            title(ax, sprintf('%d-я производная', order));
            ax.FontName = 'Times';
            ax.FontSize = 11;
        end
    end
end
