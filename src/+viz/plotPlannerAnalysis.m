function fig = plotPlannerAnalysis(result, variants, displayName)
%PLOTPLANNERANALYSIS Показывает четыре критерия и три отдельные шкалы параметров.
    screen = get(groot, 'ScreenSize');
    fig = figure('Name', ['Анализ: ', char(displayName)], 'NumberTitle', 'off', ...
        'Position', [screen(1)+40, screen(2)+40, min(1400, screen(3)-80), min(900, screen(4)-100)]);
    names = fieldnames(variants);
    metrics = {'MeanCalculationTime_s', 't, с', 'Время планирования'; ...
        'MeanPathLength_m', 'L, м', 'Длина траектории'; ...
        'MeanMaxCurvature_1_m', 'k_{max}, 1/м', 'Максимальная кривизна'; ...
        'MeanMeanCurvature_1_m', 'k_{mean}, 1/м', 'Средняя кривизна'};
    colors = lines(numel(names));
    for metric = 1:4
        left = mod(metric-1, 2)*0.5 + 0.07;
        bottom = 0.5*(metric <= 2);
        main = axes(fig, 'Position', [left, bottom+0.225, 0.385, 0.22], ...
            'Tag', sprintf('AnalysisMetric%d', metric), ...
            'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14, ...
            'LabelFontSizeMultiplier', 1, 'TitleFontSizeMultiplier', 1, ...
            'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
        hold(main, 'on');
        grid(main, 'on');
        xlim(main, [0, 1]);
        main.XTick = [];
        ylabel(main, metrics{metric, 2}, 'FontName', 'Times', ...
            'FontUnits', 'points', 'FontSize', 14, 'Color', 'k');
        title(main, metrics{metric, 3}, 'FontName', 'Times', ...
            'FontUnits', 'points', 'FontSize', 14, 'Color', 'k');
        for parameter = 1:numel(names)
            name = names{parameter};
            tableValue = result.tables.(name);
            values = tableValue.ParameterValue;
            numeric = isnumeric(values);
            logarithmic = numeric && isfield(variants.(name), 'scale') && strcmp(variants.(name).scale, 'log');
            if numeric
                coordinates = values;
                if logarithmic, coordinates = log(values); end
                if max(coordinates) == min(coordinates)
                    x = 0.5*ones(size(values));
                else
                    x = (coordinates-min(coordinates))/(max(coordinates)-min(coordinates));
                end
                ticks = values;
                labels = cellstr(compose('%.3g', values));
            else
                x = linspace(0, 1, numel(values))';
                ticks = 1:numel(values);
                if isfield(variants.(name), 'tickLabels')
                    labels = variants.(name).tickLabels;
                else
                    labels = cellstr(string(values));
                end
            end
            y = tableValue.(metrics{metric, 1});
            % Inf сохраняется в таблице и отмечается текстом, а не заменяется конечным числом.
            finiteY = y;
            finiteY(~isfinite(y)) = NaN;
            plot(main, x, finiteY, '-o', 'Color', colors(parameter, :), ...
                'LineWidth', 1.2, 'MarkerSize', 4, 'DisplayName', variants.(name).label);
            for index = find(isinf(y))'
                text(main, x(index), 0.97, 'Inf', 'Units', 'normalized', ...
                    'Color', 'k', 'HorizontalAlignment', 'center', ...
                    'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14);
            end
            scale = axes(fig, 'Position', [left, bottom+0.225-(parameter-1)*0.075, 0.385, 0.001], ...
                'Color', 'none', 'YTick', [], 'Box', 'off', ...
                'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14, ...
                'LabelFontSizeMultiplier', 1, 'TitleFontSizeMultiplier', 1, ...
                'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
                'Tag', sprintf('AnalysisParameter%d_%d', metric, parameter), ...
                'HitTest', 'off', 'PickableParts', 'none');
            scale.YAxis.Visible = 'off';
            if numel(ticks) > 1
                scale.XLim = [min(ticks), max(ticks)];
                scale.XTick = ticks;
            else
                scale.XLim = [ticks(1)-0.5, ticks(1)+0.5];
                scale.XTick = ticks;
            end
            scale.XTickLabel = labels;
            if logarithmic, scale.XScale = 'log'; end
            xlabel(scale, variants.(name).label, 'Color', 'k', ...
                'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14);
        end
        legend(main, 'Location', 'best', 'FontName', 'Times', ...
            'FontSize', 14, 'TextColor', 'k');
        hold(main, 'off');
    end
end
