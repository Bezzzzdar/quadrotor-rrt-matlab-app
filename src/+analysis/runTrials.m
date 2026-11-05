function [samples, execution] = runTrials(baseScenario, configurations, scenes, options)
%RUNTRIALS Выполняет общую очередь независимых попыток последовательно или через parfor.
% Рабочие процессы изолируют состояния RNG; seed зависит только от номера карты.
% https://www.mathworks.com/help/parallel-computing/parfor.html
% https://www.mathworks.com/help/parallel-computing/choose-between-thread-based-and-process-based-environments.html

    oldRng = rng;
    rngCleanup = onCleanup(@() rng(oldRng));
    execution = prepareExecution(options);
    showProgress = false;
    if isfield(options, 'showProgress'), showProgress = options.showProgress; end
    validateattributes(showProgress, {'logical'}, {'scalar'});
    methodCount = numel(configurations);
    trialCount = numel(scenes)*methodCount;
    configurationIndices = zeros(trialCount, 1);
    sceneIndices = zeros(trialCount, 1);
    for sceneIndex = 1:numel(scenes)
        indices = (sceneIndex - 1)*methodCount + (1:methodCount);
        configurationIndices(indices) = mod((0:methodCount-1) + sceneIndex - 1, methodCount) + 1;
        sceneIndices(indices) = sceneIndex;
    end
    template = struct('Configuration', "", 'Planner', "", 'ConfigurationIndex', 0, ...
        'SceneId', 0, 'SceneSeed', 0, 'PlannerSeed', 0, 'WorkerId', 0, ...
        'Solved', false, 'Status', "error", 'AlgorithmTime_s', NaN, ...
        'MinimumSnapTime_s', NaN, 'PlanningTime_s', NaN, 'PathLength_m', NaN, ...
        'MaxCurvature_1_m', NaN, 'MeanCurvature_1_m', NaN, 'Message', "");
    records = repmat(template, trialCount, 1);
    completed = 0;
    reportEvery = max(1, ceil(trialCount/20));
    queue = [];
    if execution.useParallel && showProgress
        queue = parallel.pool.DataQueue;
        afterEach(queue, @reportProgress);
    end
    started = tic;
    if execution.useParallel
        workerCount = execution.workerCount;
        seedBase = options.plannerSeedBase;
        % Общие карты и настройки передаются каждому процессу один раз на очередь.
        shared = parallel.pool.Constant(struct('scenario', baseScenario, ...
            'configurations', configurations, 'scenes', scenes));
        sharedCleanup = onCleanup(@() delete(shared));
        parfor (trialIndex = 1:trialCount, workerCount)
            inputs = shared.Value;
            configurationIndex = configurationIndices(trialIndex);
            records(trialIndex) = runOne(inputs.scenario, inputs.configurations(configurationIndex), ...
                inputs.scenes(sceneIndices(trialIndex)), seedBase, configurationIndex, template, true);
            if showProgress, send(queue, 1); end
        end
    else
        for trialIndex = 1:trialCount
            configurationIndex = configurationIndices(trialIndex);
            records(trialIndex) = runOne(baseScenario, configurations(configurationIndex), ...
                scenes(sceneIndices(trialIndex)), options.plannerSeedBase, configurationIndex, template, false);
            if showProgress, reportProgress([]); end
        end
    end
    execution.elapsed_s = toc(started);
    execution.trialCount = trialCount;
    samples = struct2table(records);
    if showProgress
        fprintf('Завершено %d попыток за %.1f с; режим: %s; рабочих процессов: %d.\n', ...
            trialCount, execution.elapsed_s, execution.mode, execution.workerCount);
    end

    function reportProgress(~)
    %REPORTPROGRESS Показывает выполнение очереди без вывода строки на каждую попытку.
        completed = completed + 1;
        if mod(completed, reportEvery) == 0 || completed == trialCount
            fprintf('Выполнено %d/%d (%.0f%%).\n', completed, trialCount, 100*completed/trialCount);
        end
    end
end

function record = runOne(baseScenario, configuration, scene, seedBase, configurationIndex, record, onWorker)
%RUNONE Выполняет одну попытку с восстановлением локального состояния RNG.
    oldRng = rng;
    rngCleanup = onCleanup(@() rng(oldRng));
    scenario = baseScenario;
    scenario.obstacles = scene.obstacles;
    scenario.(char(configuration.Planner)) = configuration.Parameters;
    record.Configuration = string(configuration.Name);
    record.Planner = string(configuration.Planner);
    record.ConfigurationIndex = configurationIndex;
    record.SceneId = scene.id;
    record.SceneSeed = scene.seed;
    record.PlannerSeed = seedBase + scene.id - 1;
    if onWorker
        task = getCurrentTask();
        record.WorkerId = task.ID;
    end
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
end

function execution = prepareExecution(options)
%PREPAREEXECUTION Использует пул процессов, не удаляя существующий пользовательский пул.
    execution = struct('useParallel', false, 'workerCount', 0, 'poolType', 'serial', 'mode', 'serial');
    useParallel = false;
    if isfield(options, 'useParallel'), useParallel = options.useParallel; end
    validateattributes(useParallel, {'logical'}, {'scalar'});
    if ~useParallel, return; end
    workerCount = 3;
    if isfield(options, 'maxWorkers'), workerCount = options.maxWorkers; end
    validateattributes(workerCount, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
    if exist('parpool', 'file') ~= 2 || ~license('test', 'Distrib_Computing_Toolbox')
        warning('analysis:ParallelUnavailable', ...
            'Parallel Computing Toolbox недоступен. Используется последовательный режим.');
        return;
    end
    pool = gcp('nocreate');
    if isa(pool, 'parallel.ThreadPool')
        warning('analysis:ThreadPoolUnsupported', ...
            'Открыт пул Threads. Он сохранён; для этого анализа используется последовательный режим.');
        return;
    end
    if isempty(pool)
        try
            cluster = parcluster('Processes');
            pool = parpool(cluster, min(workerCount, cluster.NumWorkers));
        catch exception
            warning('analysis:ParallelUnavailable', ...
                'Не удалось открыть пул процессов: %s. Используется последовательный режим.', exception.message);
            return;
        end
    end
    execution.useParallel = true;
    execution.workerCount = min(workerCount, pool.NumWorkers);
    execution.poolType = class(pool);
    execution.mode = 'parallel';
end
