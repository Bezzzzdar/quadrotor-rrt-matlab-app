function fig = plotPlannerComparison(result)
%PLOTPLANNERCOMPARISON Показывает распределения четырёх критериев и отдельные замеры.
% Boxplot и учёт пропусков: Moll, Sucan, Kavraki (2015), раздел III.
% https://www.kavrakilab.org/publications/moll-sucan2015benchmarking-motion-planning.pdf
    screen = get(groot, 'ScreenSize');
    fig = figure('Name', 'Сравнение методов: критерии', 'NumberTitle', 'off', ...
        'Position', [40, 40, min(1400, screen(3)-80), min(900, screen(4)-100)]);
    layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'loose', 'Padding', 'loose');
    metricNames = {'CalculationTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'};
    titles = {'Время планирования', 'Длина minimum-snap', 'Максимальная кривизна', 'Средняя кривизна'};
    units = {'t, с', 'L, м', 'k_{max}, 1/м', 'k_{mean}, 1/м'};
    names = string({result.configurations.Name});
    colors = lines(numel(names));
    times = result.samples.CalculationTime_s;
    times = times(result.samples.PlanningSolved & isfinite(times));
    logarithmicTime = ~isempty(times) && all(times > 0);
    for metric = 1:4
        ax = nexttile(layout);
        ax.Tag = sprintf('ComparisonMetric%d', metric);
        set(ax, 'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14, ...
            'LabelFontSizeMultiplier', 1, 'TitleFontSizeMultiplier', 1, ...
            'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'none');
        hold(ax, 'on');
        grid(ax, 'on');
        if metric == 1 && logarithmicTime
            ax.YScale = 'log';
            titles{metric} = 'Время планирования (лог.)';
        end
        labels = cell(numel(names), 1);
        for index = 1:numel(names)
            samples = result.samples(result.samples.Configuration == names(index), :);
            eligible = samples.Solved;
            if metric == 1, eligible = samples.PlanningSolved; end
            values = samples.(metricNames{metric});
            values = values(eligible & ~isnan(values));
            finite = values(isfinite(values));
            if ~isempty(finite)
                boxchart(ax, index*ones(size(finite)), finite, 'BoxFaceColor', colors(index, :), ...
                    'BoxEdgeColor', 'k', 'BoxMedianLineColor', 'k', 'WhiskerLineColor', 'k', ...
                    'MarkerStyle', 'none', 'LineWidth', 1.1);
                offsets = zeros(size(finite));
                if numel(finite) > 1, offsets = linspace(-0.12, 0.12, numel(finite))'; end
                scatter(ax, index+offsets, finite, 22, colors(index, :), 'filled', ...
                    'Tag', sprintf('ComparisonPoints%d_%d', metric, index));
            end
            labels{index} = sprintf('%s (n=%d/%d)', names(index), numel(values), height(samples));
            if any(isinf(values))
                text(ax, (index-0.5)/numel(names), 0.97, sprintf('Inf: %d', sum(isinf(values))), ...
                    'Units', 'normalized', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
                    'FontName', 'Times', 'FontSize', 14, 'Color', 'k');
            end
        end
        xlim(ax, [0.5, numel(names)+0.5]);
        ax.XTick = 1:numel(names);
        ax.XTickLabel = labels;
        ax.XTickLabelRotation = 0;
        title(ax, titles{metric}, 'FontName', 'Times', 'FontSize', 14, 'Color', 'k');
        ylabel(ax, units{metric}, 'FontName', 'Times', 'FontSize', 14, 'Color', 'k');
        hold(ax, 'off');
    end
end
