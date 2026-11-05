function study = runParameterStudy(baseScenario, configurations, variants, options, scenes)
%RUNPARAMETERSTUDY Исследует по одному параметру на одном наборе случайных карт.
% Все независимые попытки образуют общую очередь без вложенных параллельных циклов.
% Прочие параметры сохраняют базовые значения; моменты условны по допустимым траекториям.
% Исправленная дисперсия: https://www.itl.nist.gov/div898/handbook/eda/section3/eda356.htm

    if nargin < 5
        scenes = analysis.validateExperiment(baseScenario, configurations, options);
    else
        scenes = analysis.validateExperiment(baseScenario, configurations, options, scenes);
    end
    groupCount = sum(arrayfun(@(configuration) numel(fieldnames(variants.(char(configuration.Planner)))), configurations));
    groups = repmat(struct('Planner', '', 'Parameter', '', 'Values', [], ...
        'ConfigurationIndices', []), groupCount, 1);
    groupIndex = 0;
    configurationSets = cell(groupCount, 1);
    configurationCount = 0;
    for methodIndex = 1:numel(configurations)
        baseline = configurations(methodIndex);
        planner = char(baseline.Planner);
        parameters = fieldnames(variants.(planner));
        for parameterIndex = 1:numel(parameters)
            parameter = parameters{parameterIndex};
            if ~isfield(baseline.Parameters, parameter)
                error('analysis:InvalidParameter', 'У метода %s отсутствует параметр %s.', planner, parameter);
            end
            values = variants.(planner).(parameter).value(:);
            if isempty(values) || ~(isnumeric(values) || isstring(values)) || ...
                    numel(unique(values)) ~= numel(values)
                error('analysis:InvalidParameterValues', 'Нужен непустой набор различных чисел или строк.');
            end
            indices = configurationCount + (1:numel(values));
            configurationCount = configurationCount + numel(values);
            groupIndex = groupIndex + 1;
            groups(groupIndex) = struct('Planner', planner, 'Parameter', parameter, ...
                'Values', values, 'ConfigurationIndices', indices);
            settings = repmat(baseline, numel(values), 1);
            for valueIndex = 1:numel(values)
                settings(valueIndex).Parameters.(parameter) = values(valueIndex);
            end
            configurationSets{groupIndex} = settings;
        end
    end
    taskConfigurations = vertcat(configurationSets{:});
    [samples, execution] = analysis.runTrials(baseScenario, taskConfigurations, scenes, options);
    study = struct('methods', struct(), 'scenes', scenes, 'scenario', baseScenario, ...
        'configurations', configurations, 'variants', variants, 'options', options, 'execution', execution);
    metricNames = {'PlanningTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'};
    for group = groups'
        count = numel(group.Values);
        summary = table(group.Values, zeros(count, 1), zeros(count, 1), nan(count, 1), ...
            'VariableNames', {'ParameterValue', 'AttemptCount', 'SuccessCount', 'Probability'});
        for metric = 1:numel(metricNames)
            summary.(['Mean', metricNames{metric}]) = nan(count, 1);
            summary.(['Variance', metricNames{metric}]) = nan(count, 1);
        end
        attempts = cell(count, 1);
        for valueIndex = 1:count
            configurationIndex = group.ConfigurationIndices(valueIndex);
            selected = samples(samples.ConfigurationIndex == configurationIndex, :);
            [tables, probabilities] = analysis.summarizeResults(selected, taskConfigurations(configurationIndex));
            moments = tables.(group.Planner);
            summary.AttemptCount(valueIndex) = height(selected);
            summary.SuccessCount(valueIndex) = sum(selected.Solved);
            summary.Probability(valueIndex) = probabilities.(group.Planner);
            for metric = 1:numel(metricNames)
                summary.(['Mean', metricNames{metric}])(valueIndex) = moments.Mean(metric);
                summary.(['Variance', metricNames{metric}])(valueIndex) = moments.Variance(metric);
            end
            selected.ValueIndex = repmat(valueIndex, height(selected), 1);
            selected.ParameterValue = repmat(group.Values(valueIndex), height(selected), 1);
            attempts{valueIndex} = selected;
        end
        study.methods.(group.Planner).(group.Parameter) = struct('table', summary, ...
            'samples', vertcat(attempts{:}), 'configurations', taskConfigurations(group.ConfigurationIndices));
    end
end
