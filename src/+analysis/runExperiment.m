function result = runExperiment(baseScenario, configurations, options)
%RUNEXPERIMENT Сравнивает методы с фиксированными параметрами на общих картах.
% Успехом считается только геометрически допустимая minimum-snap траектория.
% Время планирования равно времени алгоритма плюс времени построения minimum-snap.
% Подготовка маршрута, проверки, метрики и динамика в это время не входят.

    scenes = analysis.generateScenes(baseScenario, options);
    validateattributes(options.plannerSeedBase, {'numeric'}, ...
        {'scalar', 'integer', 'nonnegative', '<=', 2^32-options.sceneCount});
    names = string({configurations.Name});
    planners = string({configurations.Planner});
    if isempty(names) || any(strlength(names) == 0) || numel(unique(names)) ~= numel(names) || ...
            numel(unique(planners)) ~= numel(planners) || ...
            any(~ismember(planners, ["rrt", "astar", "voronoi"]))
        error('analysis:InvalidConfigurations', 'Нужны уникальные названия и методы rrt, astar или voronoi.');
    end
    for index = 1:numel(configurations)
        if ~isstruct(configurations(index).Parameters) || ~isscalar(configurations(index).Parameters)
            error('analysis:InvalidConfigurations', 'Параметры метода должны быть скалярной структурой.');
        end
    end
    oldRng = rng;
    rngCleanup = onCleanup(@() rng(oldRng));
    template = struct('Configuration', "", 'Planner', "", 'SceneId', 0, ...
        'SceneSeed', 0, 'PlannerSeed', 0, 'Solved', false, 'Status', "error", ...
        'AlgorithmTime_s', NaN, 'MinimumSnapTime_s', NaN, 'PlanningTime_s', NaN, ...
        'PathLength_m', NaN, 'MaxCurvature_1_m', NaN, 'MeanCurvature_1_m', NaN, ...
        'Message', "");
    records = repmat(template, options.sceneCount*numel(configurations), 1);
    recordIndex = 0;
    for sceneIndex = 1:options.sceneCount
        % Порядок циклически меняется, но одновременно выполняется только один метод.
        order = mod((0:numel(configurations)-1)+sceneIndex-1, numel(configurations))+1;
        for index = order
            configuration = configurations(index);
            scenario = baseScenario;
            scenario.obstacles = scenes(sceneIndex).obstacles;
            scenario.(char(configuration.Planner)) = configuration.Parameters;
            record = template;
            record.Configuration = string(configuration.Name);
            record.Planner = string(configuration.Planner);
            record.SceneId = scenes(sceneIndex).id;
            record.SceneSeed = scenes(sceneIndex).seed;
            record.PlannerSeed = options.plannerSeedBase + sceneIndex - 1;
            rng(record.PlannerSeed, 'twister');
            try
                experiment = runPlannerExperiment(scenario, configuration.Planner, false);
                record.Solved = experiment.pathFound;
                record.Status = string(experiment.status);
                record.AlgorithmTime_s = experiment.calculationTime;
                record.MinimumSnapTime_s = experiment.minimumSnapTime;
                record.PlanningTime_s = experiment.planningTime;
                if record.Solved
                    record.PathLength_m = experiment.metrics.pathLength;
                    record.MaxCurvature_1_m = experiment.metrics.maxCurvature;
                    record.MeanCurvature_1_m = experiment.metrics.meanCurvature;
                end
                record.Message = string(experiment.message);
            catch exception
                record.Message = string(exception.identifier) + ": " + string(exception.message);
            end
            recordIndex = recordIndex + 1;
            records(recordIndex) = record;
        end
    end
    samples = struct2table(records);
    [tables, probabilities] = analysis.summarizeResults(samples, configurations);
    result = struct('tables', tables, 'probabilities', probabilities, ...
        'samples', samples, 'scenes', scenes, 'scenario', baseScenario, ...
        'configurations', configurations, 'options', options);
end
