function result = runExperiment(baseScenario, configurations, options, scenes)
%RUNEXPERIMENT Сравнивает фиксированные методы на общих случайных картах.
% Успех - геометрически допустимая minimum-snap траектория.
% Время планирования равно времени алгоритма плюс времени построения minimum-snap.

    if nargin < 4
        scenes = analysis.validateExperiment(baseScenario, configurations, options);
    else
        scenes = analysis.validateExperiment(baseScenario, configurations, options, scenes);
    end
    [samples, execution] = analysis.runTrials(baseScenario, configurations, scenes, options);
    [tables, probabilities] = analysis.summarizeResults(samples, configurations);
    result = struct('tables', tables, 'probabilities', probabilities, ...
        'samples', samples, 'scenes', scenes, 'scenario', baseScenario, ...
        'configurations', configurations, 'options', options, 'execution', execution);
end
