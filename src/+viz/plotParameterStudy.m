function fig = plotParameterStudy(study, planner)
%PLOTPARAMETERSTUDY Показывает критерии и вероятность на пяти вкладках метода.
% Среднее и дисперсия расположены рядом и имеют по три независимые оси X.
% Служебные координаты кривых согласуются с исходными шкалами параметров.

    planner = char(planner);
    configuration = study.configurations(strcmp({study.configurations.Planner}, planner));
    parameters = fieldnames(study.methods.(planner));
    fig = figure('Name', configuration.Name, 'NumberTitle', 'off', 'Color', 'w', ...
        'Position', [50, 50, 1600, 1000], 'Tag', ['ParameterStudy_', planner]);
    tabs = uitabgroup(fig, 'Units', 'normalized', 'Position', [0, 0, 1, 1], ...
        'Tag', 'StudyTabs', 'SelectionChangedFcn', @selectTab);
    metrics = {'PlanningTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'};
    titles = {'Время планирования', 'Длина траектории', 'Максимальная кривизна', 'Средняя кривизна'};
    meanUnits = {'M_{x}', 'M_{x}', 'M_{x}', 'M_{x}'};
    varianceUnits = {'D_{x}', 'D_{x}', 'D_{x}', 'D_{x}'};
    for metric = 1:numel(metrics)
        tab = uitab(tabs, 'Title', titles{metric}, 'Tag', metrics{metric});
        createField(tab, [0, 0, 0.5, 1], study, planner, parameters, ...
            ['Mean', metrics{metric}], '', meanUnits{metric});
        createField(tab, [0.5, 0, 0.5, 1], study, planner, parameters, ...
            ['Variance', metrics{metric}], '', varianceUnits{metric});
    end
    tab = uitab(tabs, 'Title', 'Вероятность решения задачи', 'Tag', 'Probability');
    createField(tab, [0, 0, 1, 1], study, planner, parameters, ...
        'Probability', '', 'W');
    fig.SizeChangedFcn = @layoutFields;
    layoutFields(fig, []);
end

function createField(tab, position, study, planner, parameters, column, heading, unitLabel)
%CREATEFIELD Накладывает три серии и создаёт отдельную шкалу X для каждой серии.
    panel = uipanel(tab, 'Units', 'normalized', 'Position', position, ...
        'BorderType', 'none', 'BackgroundColor', 'w', 'Tag', ['StudyField_', column]);
    panel.UserData = position;
    if isprop(tab, 'AutoResizeChildren'), tab.AutoResizeChildren = 'off'; end
    if isprop(panel, 'AutoResizeChildren'), panel.AutoResizeChildren = 'off'; end
    ax = axes(panel, 'Units', 'pixels', 'PositionConstraint', 'innerposition', 'Tag', 'StudyDataAxes');
    styleAxes(ax);
    ax.UserData = struct('Column', column);
    ax.XAxis.Visible = 'off';
    ax.XLim = [0, 1];
    ax.YAxis.Exponent = 0;
    ax.YGrid = 'on';
    ax.XGrid = 'on';
    ax.Box = 'on';
    values = cellfun(@(parameter) study.methods.(planner).(parameter).table.(column), ...
        parameters, 'UniformOutput', false);
    ax.YLim = valueLimits(vertcat(values{:}), strcmp(column, 'Probability'));
    categorical = cellfun(@(parameter) ...
        ~isnumeric(study.methods.(planner).(parameter).table.ParameterValue), parameters);
    categoryIndices = find(categorical);
    if any(categorical), ax.YLim(1) = 0; end
    title(ax, heading, 'FontWeight', 'bold', 'Interpreter', 'tex');
    ylabel(ax, unitLabel, 'Interpreter', 'tex');
    hold(ax, 'on');
    colors = [0, 0.35, 0.65; 0.75, 0.25, 0.12; 0.12, 0.50, 0.32];
    series = gobjects(numel(parameters), 1);
    names = cell(numel(parameters), 1);
    for index = 1:numel(parameters)
        parameter = parameters{index};
        variant = study.variants.(planner).(parameter);
        summary = study.methods.(planner).(parameter).table;
        offset = 0;
        if categorical(index)
            rank = find(categoryIndices == index);
            offset = (rank - (numel(categoryIndices) + 1)/2) * ...
                0.64/(numel(categoryIndices)*height(summary));
        end
        [x, mapping, order] = parameterCoordinates(summary.ParameterValue, variant, offset);
        original = summary.(column)(order);
        displayed = original;
        displayed(~isfinite(displayed)) = NaN;
        if categorical(index)
            series(index) = bar(ax, x, displayed, 0.56/numel(categoryIndices), ...
                'FaceColor', colors(index, :), 'EdgeColor', colors(index, :), ...
                'FaceAlpha', 0.35, 'LineWidth', 1.2, 'Tag', ['StudySeries_', parameter]);
        else
            series(index) = plot(ax, x, displayed, 'LineStyle', '-', ...
                'Marker', 'none', 'Color', colors(index, :), ...
                'LineWidth', 1.2, 'Clipping', 'on', 'Tag', ['StudySeries_', parameter]);
        end
        series(index).UserData = struct('ParameterValue', summary.ParameterValue(order), 'Column', column);
        series(index).DataTipTemplate.DataTipRows = [dataTipTextRow(variant.label, summary.ParameterValue(order)), ...
            dataTipTextRow(unitLabel, original)];
        names{index} = variant.label;
        scale = axes(panel, 'Units', 'pixels', 'PositionConstraint', 'innerposition', ...
            'Color', 'none', 'Box', 'off', 'HitTest', 'off', 'PickableParts', 'none', ...
            'HandleVisibility', 'off', 'Tag', ['StudyParameterAxis_', parameter]);
        styleAxes(scale);
        mapping.Index = index;
        scale.UserData = mapping;
        scale.YAxis.Visible = 'off';
        scale.XScale = mapping.Scale;
        xlabel(scale, variant.label, 'Interpreter', 'tex');
        applyScaleLimits(scale, [0, 1]);
        if all(isnan(original))
            text(ax, 0.02, 0.96 - 0.06*(index - 1), ['Нет оценки: ', variant.label], ...
                'Units', 'normalized', 'VerticalAlignment', 'top', 'Interpreter', 'tex', ...
                'Tag', 'StudyMissingEstimate');
        end
        for point = find(isinf(original))'
            label = 'Inf';
            if original(point) < 0, label = '-Inf'; end
            text(ax, x(point), ax.YLim(2), label, 'VerticalAlignment', 'top', ...
                'HorizontalAlignment', 'center', 'Interpreter', 'tex', 'Tag', 'StudyInfiniteEstimate');
        end
    end
    hold(ax, 'off');
    if any(categorical), uistack(series(~categorical), 'top'); end
    legend(ax, series, names, 'Location', 'northeast', 'Interpreter', 'tex', ...
        'Box', 'on', 'FontName', 'Times', 'FontSize', 14, 'TextColor', 'k', ...
        'Tag', 'StudyLegend');
    ax.XAxis.LimitsChangedFcn = @synchronizeScales;
    set(findall(ax, 'Type', 'text'), 'Color', 'k', ...
        'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14);
end

function [x, mapping, order] = parameterCoordinates(values, variant, offset)
%PARAMETERCOORDINATES Переводит положение серии в общую область без изменения её значений.
    mapping = struct('Scale', 'linear', 'Coordinates', [], 'Ticks', [], 'Labels', {{}}, 'Categorical', false);
    order = (1:numel(values))';
    if ~isnumeric(values)
        coordinates = order;
        % Смещение разделяет наборы столбцов; их собственные оси остаются согласованы.
        mapping.Coordinates = [0.5, numel(values) + 0.5] - offset*numel(values);
        mapping.Ticks = coordinates;
        mapping.Labels = cellstr(values);
        mapping.Categorical = true;
        if isfield(variant, 'tickLabels'), mapping.Labels = variant.tickLabels; end
    else
        [values, order] = sort(values);
        coordinates = values;
        mapping.Coordinates = coordinates([1, end])';
        if isscalar(values)
            padding = max(1, 0.05*abs(coordinates));
            mapping.Coordinates = coordinates + [-padding, padding];
        end
        ticks = unique(round(linspace(1, numel(values), min(5, numel(values)))));
        mapping.Ticks = values(ticks);
        % Неравномерная выборка параметра не должна создавать тесные подписи оси.
        if ~isscalar(values) && ...
                max(diff(values)) - min(diff(values)) > sqrt(eps)*diff(values([1, end]))
            mapping.Ticks = linspace(values(1), values(end), min(5, numel(values)));
        end
        mapping.Labels = cellstr(compose('%.6g', mapping.Ticks));
    end
    x = (coordinates - mapping.Coordinates(1))/diff(mapping.Coordinates);
end

function applyScaleLimits(scale, normalizedLimits)
%APPLYSCALELIMITS Согласует исходную шкалу параметра с текущим увеличением поля.
    mapping = scale.UserData;
    limits = mapping.Coordinates(1) + normalizedLimits*diff(mapping.Coordinates);
    scale.XLim = limits;
    if mapping.Categorical || isequal(normalizedLimits, [0, 1])
        scale.XTick = mapping.Ticks;
        scale.XTickLabel = mapping.Labels;
    else
        scale.XTickMode = 'auto';
        scale.XTickLabelMode = 'auto';
    end
    if strcmp(mapping.Scale, 'linear'), scale.XAxis.Exponent = 0; end
end

function synchronizeScales(ruler, ~)
%SYNCHRONIZESCALES Сохраняет соответствие трёх осей при увеличении и перемещении.
    ax = ancestor(ruler, 'axes');
    scales = findall(ax.Parent, '-regexp', 'Tag', '^StudyParameterAxis_');
    for scale = scales'
        applyScaleLimits(scale, ax.XLim);
    end
end

function selectTab(group, ~)
%SELECTTAB Обновляет геометрию при открытии вкладки, ранее скрытой MATLAB.
    layoutFields(ancestor(group, 'figure'), []);
end

function layoutFields(fig, ~)
%LAYOUTFIELDS Выравнивает все три горизонтальные оси по краям поля и разносит по высоте.
    figureBounds = getpixelposition(fig);
    % Собственная геометрия не зависит от задержки обновления нативной вкладки.
    contentSize = [figureBounds(3), figureBounds(4) - 30];
    panels = findall(fig, '-regexp', 'Tag', '^StudyField_');
    for panel = panels'
        panel.Units = 'pixels';
        panel.Position = panel.UserData .* [contentSize, contentSize];
        bounds = getpixelposition(panel);
        if bounds(3) < 200 || bounds(4) < 450, continue; end
        left = 110;
        width = bounds(3) - left - 28;
        bottom = 252;
        height = bounds(4) - bottom - 145;
        ax = findall(panel, 'Tag', 'StudyDataAxes');
        ax.Position = [left, bottom, width, height];
        ax.Title.Units = 'pixels';
        ax.Title.Position = [width/2, height + 125, 0];
        ax.Title.VerticalAlignment = 'top';
        key = findall(panel, 'Tag', 'StudyLegend');
        key.Units = 'pixels';
        key.Location = 'northeast';
        scales = findall(panel, '-regexp', 'Tag', '^StudyParameterAxis_');
        for scale = scales'
            scale.Position = [left, bottom - 86*(scale.UserData.Index - 1), width, 1];
        end
    end
end

function limits = valueLimits(values, probability)
%VALUELIMITS Задаёт общую шкалу Y для трёх серий, сохраняя отсутствие оценок.
    limits = [0, 1];
    values = values(isfinite(values));
    if probability || isempty(values), return; end
    lower = min(values);
    upper = max(values);
    span = upper - lower;
    if span <= sqrt(eps)*max(abs(values))
        limits = [min(0, lower), 1.08*max(0, upper)];
        if limits(1) == limits(2), limits = [0, 1]; end
    else
        limits = [max(0, lower - 0.08*span), upper + 0.08*span];
    end
end

function styleAxes(ax)
%STYLEAXES Сохраняет чёрные оси и подписи шрифтом Times размером 14 пунктов.
    set(ax, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
        'FontName', 'Times', 'FontUnits', 'points', 'FontSize', 14, ...
        'TitleFontSizeMultiplier', 1, 'LabelFontSizeMultiplier', 1, 'TickLabelInterpreter', 'tex');
end
