function scenes = validateExperiment(baseScenario, configurations, options, scenes)
%VALIDATEEXPERIMENT Проверяет общие карты, начальные состояния генераторов и методы.

    if nargin < 4
        scenes = analysis.generateScenes(baseScenario, options);
    else
        validateattributes(options.sceneCount, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
        validateattributes(options.sceneSeedBase, {'numeric'}, ...
            {'scalar', 'integer', 'nonnegative', '<=', 2^32-options.sceneCount});
        if ~isstruct(scenes) || numel(scenes) ~= options.sceneCount || ...
                ~all(isfield(scenes, {'id', 'seed', 'obstacles'}))
            error('analysis:InvalidScenes', 'Нужен общий набор из sceneCount случайных карт.');
        end
        for sceneIndex = 1:numel(scenes)
            if ~isequal(scenes(sceneIndex).id, sceneIndex) || ...
                    ~isequal(scenes(sceneIndex).seed, options.sceneSeedBase + sceneIndex - 1)
                error('analysis:InvalidScenes', 'Номера и seed карт не соответствуют настройкам анализа.');
            end
            geometry.validateScenario(baseScenario.map, baseScenario.start2d, ...
                baseScenario.goal2d, scenes(sceneIndex).obstacles, baseScenario.safetyRadius);
        end
    end
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
end
