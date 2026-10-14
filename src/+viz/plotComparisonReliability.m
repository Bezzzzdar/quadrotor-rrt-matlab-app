function fig = plotComparisonReliability(result)
%PLOTCOMPARISONRELIABILITY Строит ECDF с неудачами в знаменателе всех запусков.
% F(t) = число допустимых решений, найденных не позднее t / число попыток.
% https://www.kavrakilab.org/publications/moll-sucan2015benchmarking-motion-planning.pdf
    screen = get(groot, 'ScreenSize');
    fig = figure('Name', 'Сравнение методов: вероятность решения', 'NumberTitle', 'off', ...
        'Position', [60, 60, min(1400, screen(3)-100), min(650, screen(4)-120)]);
    layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'loose', 'Padding', 'loose');
    names = string({result.configurations.Name});
    colors = lines(numel(names));
    fields = {'CalculationTime_s', 'PipelineTime_s'};
    flags = {'PlanningSolved', 'Solved'};
    titles = {'Допустимый геометрический путь', 'Допустимая minimum-snap'};
    for panel = 1:2
        ax = nexttile(layout);
        ax.Tag = sprintf('ComparisonECDF%d', panel);
        set(ax, 'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14, ...
            'LabelFontSizeMultiplier', 1, 'TitleFontSizeMultiplier', 1, 'XColor', 'k', 'YColor', 'k');
        hold(ax, 'on');
        grid(ax, 'on');
        times = result.samples.(fields{panel});
        known = times(result.samples.(flags{panel}) & isfinite(times));
        right = result.metadata.options.timeLimit_s;
        if ~isempty(known) && ~any(result.samples.Status == "timeout")
            right = max(known)*1.1;
        elseif ~isempty(known)
            right = max(right, max(known)*1.1);
        end
        right = max(right, eps);
        for index = 1:numel(names)
            samples = result.samples(result.samples.Configuration == names(index), :);
            times = samples.(fields{panel});
            times = times(samples.(flags{panel}) & isfinite(times));
            [values, ~, groups] = unique(times);
            probability = zeros(0, 1);
            if ~isempty(values), probability = cumsum(accumarray(groups, 1))/height(samples); end
            last = 0;
            if ~isempty(probability), last = probability(end); end
            stairs(ax, [0; values; right], [0; probability; last], ...
                'LineWidth', 1.5, 'Color', colors(index, :), 'DisplayName', char(names(index)), ...
                'Tag', sprintf('ComparisonCDF%d_%d', panel, index));
        end
        xlim(ax, [0, right]);
        ylim(ax, [0, 1]);
        title(ax, titles{panel}, 'FontName', 'Times', 'FontSize', 14, 'Color', 'k');
        xlabel(ax, 'Время, с', 'FontName', 'Times', 'FontSize', 14, 'Color', 'k');
        ylabel(ax, 'Доля успешных запусков', 'FontName', 'Times', 'FontSize', 14, 'Color', 'k');
        legend(ax, 'Location', 'southeast', 'FontName', 'Times', 'FontSize', 14, 'TextColor', 'k');
        hold(ax, 'off');
    end
end
