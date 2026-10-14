function result = runMethod(baseScenario, plannerName, variants, runsPerValue, seedBase, progress)
%RUNMETHOD Меняет один параметр метода и повторяет каждое значение заданное число раз.
% Время означает время планирования; длина и кривизна относятся к minimum-snap.
% Кривизна: https://arxiv.org/pdf/math/0606007
    if nargin < 6, progress = []; end
    validateattributes(runsPerValue, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
    validateattributes(seedBase, {'numeric'}, {'scalar', 'integer', 'nonnegative', 'finite'});
    plannerName = char(lower(string(plannerName)));
    if ~ismember(plannerName, {'rrt', 'astar', 'voronoi'})
        error('analysis:InvalidPlanner', 'Неизвестный метод анализа.');
    end
    geometry.validateScenario(baseScenario.map, baseScenario.start2d, baseScenario.goal2d, ...
        baseScenario.obstacles, baseScenario.safetyRadius);
    names = fieldnames(variants);
    result = struct('plannerName', plannerName, 'tables', struct(), 'samples', struct(), ...
        'startedAt', datetime('now', 'TimeZone', 'UTC'), 'finishedAt', [], 'workerTaskID', NaN);
    task = getCurrentTask();
    if ~isempty(task), result.workerTaskID = task.ID; end
    % Прогрев кода и решателя исключён из повторных замеров.
    rng(seedBase, 'twister');
    runPlannerExperiment(baseScenario, plannerName, false);

    for parameter = 1:numel(names)
        name = names{parameter};
        values = variants.(name).value(:);
        count = numel(values);
        timing = nan(count, runsPerValue);
        lengthValue = timing;
        maximum = timing;
        average = timing;
        found = false(count, runsPerValue);
        messages = strings(count, runsPerValue);
        errorCount = zeros(count, 1);
        for index = 1:count
            scenario = baseScenario;
            scenario.(plannerName).(name) = values(index);
            for repeat = 1:runsPerValue
                % Одинаковый набор seed для всех значений, без rng('shuffle').
                rng(seedBase + repeat - 1, 'twister');
                try
                    experiment = runPlannerExperiment(scenario, plannerName, false);
                    timing(index, repeat) = experiment.calculationTime;
                    found(index, repeat) = experiment.pathFound;
                    if experiment.pathFound
                        lengthValue(index, repeat) = experiment.metrics.pathLength;
                        maximum(index, repeat) = experiment.metrics.maxCurvature;
                        average(index, repeat) = experiment.metrics.meanCurvature;
                    else
                        messages(index, repeat) = string(experiment.message);
                    end
                catch exception
                    errorCount(index) = errorCount(index) + 1;
                    messages(index, repeat) = string(exception.identifier) + ": " + string(exception.message);
                end
            end
            if ~isempty(progress)
                send(progress, struct('planner', plannerName, 'parameter', name, ...
                    'value', char(string(values(index))), 'index', index, 'count', count, ...
                    'found', sum(found(index, :)), 'runs', runsPerValue));
            end
        end
        messageSummary = strings(count, 1);
        for index = 1:count
            selected = messages(index, messages(index, :) ~= "");
            if ~isempty(selected)
                messageSummary(index) = join(unique(selected, 'stable'), "; ");
            end
        end
        result.tables.(name) = table(values, ...
            mean(timing, 2, 'omitnan'), mean(lengthValue, 2, 'omitnan'), ...
            mean(maximum, 2, 'omitnan'), mean(average, 2, 'omitnan'), ...
            mean(found, 2), errorCount, sum(isinf(maximum), 2), messageSummary, ...
            'VariableNames', {'ParameterValue', 'MeanCalculationTime_s', 'MeanPathLength_m', ...
            'MeanMaxCurvature_1_m', 'MeanMeanCurvature_1_m', 'PathFoundRate', ...
            'ErrorCount', 'UnboundedRuns', 'Message'});
        result.samples.(name) = table(repelem(values, runsPerValue), ...
            repmat((1:runsPerValue)', count, 1), reshape(timing', [], 1), ...
            reshape(lengthValue', [], 1), reshape(maximum', [], 1), reshape(average', [], 1), ...
            reshape(found', [], 1), reshape(messages', [], 1), ...
            'VariableNames', {'ParameterValue', 'Repeat', 'CalculationTime_s', 'PathLength_m', ...
            'MaxCurvature_1_m', 'MeanCurvature_1_m', 'PathFound', 'Message'});
    end
    result.finishedAt = datetime('now', 'TimeZone', 'UTC');
end
