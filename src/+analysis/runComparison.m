function result = runComparison(baseScenario, configurations, options)
%RUNCOMPARISON Сравнивает фиксированные конфигурации последовательными замерами.
% Протокол повторов, исходов и метаданных: Moll, Sucan, Kavraki (2015), разделы II-III.
% https://www.kavrakilab.org/publications/moll-sucan2015benchmarking-motion-planning.pdf
    validateattributes(options.runCount, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
    validateattributes(options.seedBase, {'numeric'}, ...
        {'scalar', 'integer', 'nonnegative', '<=', 2^32-options.runCount});
    validateattributes(options.timeLimit_s, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    geometry.validateScenario(baseScenario.map, baseScenario.start2d, baseScenario.goal2d, ...
        baseScenario.obstacles, baseScenario.safetyRadius);
    names = string({configurations.Name});
    if isempty(names) || any(strlength(names) == 0) || numel(unique(names)) ~= numel(names)
        error('comparison:InvalidNames', 'Названия конфигураций должны быть непустыми и уникальными.');
    end
    for index = 1:numel(configurations)
        method = char(configurations(index).Planner);
        if ~ismember(method, {'rrt', 'astar', 'voronoi'}) || ~isstruct(configurations(index).Parameters)
            error('comparison:InvalidConfiguration', 'Неверный метод или структура параметров.');
        end
    end
    oldRng = rng;
    rngCleanup = onCleanup(@() rng(oldRng));
    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool('Processes', 1);
        poolCleanup = onCleanup(@() delete(pool));
    end
    if isa(pool, 'parallel.ThreadPool')
        error('comparison:ProcessPoolRequired', 'Для отмены изолированных задач нужен пул Processes.');
    end
    requireIdlePool(pool);
    metadata = struct('startedAt', datetime('now', 'TimeZone', 'UTC'), ...
        'finishedAt', [], 'options', options, 'matlabVersion', version, ...
        'platform', computer, 'hostname', getenv('COMPUTERNAME'), ...
        'cpu', getenv('PROCESSOR_IDENTIFIER'), 'logicalProcessors', getenv('NUMBER_OF_PROCESSORS'), ...
        'poolWorkers', pool.NumWorkers, 'toolboxes', ver, 'memoryLimitBytes', NaN, ...
        'source', 'https://www.kavrakilab.org/publications/moll-sucan2015benchmarking-motion-planning.pdf', ...
        'timeMeaning', 'CalculationTime: поиск; WorkerTime: задача; PipelineTime: поиск + обработка + проверка.', ...
        'limitMeaning', 'Общий бюджет задачи геометрического планирования без ожидания очереди; не лимит minimum-snap.', ...
        'qualityMeaning', 'Длина и кривизна только допустимой minimum-snap; время только допустимого геометрического пути.', ...
        'validationMeaning', 'Непрерывные полиномы, корни пересечений; численный допуск 1e-9*max(1,width,height).');
    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    files = [dir(fullfile(rootDir, 'src', '**', '*.m')); dir(fullfile(rootDir, 'scripts', 'run_comparison.m'))];
    snapshot = repmat(struct('relativePath', '', 'text', ''), numel(files), 1);
    for index = 1:numel(files)
        file = fullfile(files(index).folder, files(index).name);
        snapshot(index).relativePath = erase(file, [rootDir, filesep]);
        snapshot(index).text = fileread(file);
    end
    metadata.sourceSnapshot = snapshot;
    metadata.warmupStatus = strings(numel(configurations), 1);
    % Прогреваются все работники: следующий parfeval может выбрать любого из них.
    for index = 1:numel(configurations)
        scenario = configuredScenario(baseScenario, configurations(index));
        future = parfevalOnAll(pool, @analysis.planSeeded, 1, ...
            scenario, configurations(index).Planner, options.seedBase);
        cleanup = onCleanup(@() cancel(future));
        if ~wait(future, 'finished', 60)
            error('comparison:WarmupTimeout', 'Прогрев превысил 60 секунд.');
        end
        try
            warmPlans = fetchOutputs(future);
            if iscell(warmPlans)
                warmPlan = warmPlans{1};
            else
                warmPlan = warmPlans(1);
            end
            % Обработка и непрерывная проверка выполняются на клиенте, как и в замерах.
            warmup = finishTrial(struct(), warmPlan(1), scenario);
            metadata.warmupStatus(index) = warmup.Status;
        catch exception
            metadata.warmupStatus(index) = "error";
            warning('comparison:WarmupError', 'Ошибка прогрева %s: %s', ...
                configurations(index).Name, exception.message);
        end
    end
    template = struct('Configuration', "", 'Planner', "", 'Repeat', 0, 'Seed', 0, ...
        'StartedAt', NaT('TimeZone', 'UTC'), 'FinishedAt', NaT('TimeZone', 'UTC'), ...
        'Status', "error", 'PlanningSolved', false, 'Solved', false, ...
        'CalculationTime_s', NaN, 'WorkerTime_s', NaN, 'PostprocessingTime_s', NaN, ...
        'ValidationTime_s', NaN, 'PipelineTime_s', NaN, 'RawPathLength_m', NaN, ...
        'PathLength_m', NaN, 'MaxCurvature_1_m', NaN, 'MeanCurvature_1_m', NaN, ...
        'Message', "", 'ErrorIdentifier', "");
    count = numel(configurations)*options.runCount;
    records = repmat(template, count, 1);
    paths = cell(count, 1);
    recordIndex = 0;
    for repeat = 1:options.runCount
        % Циклический порядок уменьшает зависимость измерений от позиции метода в серии.
        order = mod((0:numel(configurations)-1)+repeat-1, numel(configurations))+1;
        for index = order
            requireIdlePool(pool);
            configuration = configurations(index);
            scenario = configuredScenario(baseScenario, configuration);
            record = template;
            record.Configuration = string(configuration.Name);
            record.Planner = string(configuration.Planner);
            record.Repeat = repeat;
            record.Seed = options.seedBase+repeat-1;
            record.StartedAt = datetime('now', 'TimeZone', 'UTC');
            [planned, status, message, workerTime, errorID] = analysis.runTimedPlan( ...
                pool, scenario, configuration.Planner, record.Seed, options.timeLimit_s);
            record.Status = status;
            record.Message = message;
            record.ErrorIdentifier = errorID;
            record.WorkerTime_s = workerTime;
            storedPath = [];
            if status == "completed"
                [record, storedPath] = finishTrial(record, planned, scenario);
            end
            record.FinishedAt = datetime('now', 'TimeZone', 'UTC');
            recordIndex = recordIndex+1;
            records(recordIndex) = record;
            paths{recordIndex} = storedPath;
            fprintf('[%s] %d/%d, seed=%d: %s\n', record.Configuration, ...
                repeat, options.runCount, record.Seed, record.Status);
        end
    end
    metadata.finishedAt = datetime('now', 'TimeZone', 'UTC');
    result = struct('samples', struct2table(records), 'paths', {paths}, ...
        'configurations', configurations, 'scenario', baseScenario, 'metadata', metadata);
    [result.summary, result.statistics] = analysis.summarizeComparison(result.samples, configurations);
end

function scenario = configuredScenario(baseScenario, configuration)
%CONFIGUREDSCENARIO Меняет только настройки метода, не карту и условия траектории.
    scenario = baseScenario;
    scenario.(char(configuration.Planner)) = configuration.Parameters;
end

function requireIdlePool(pool)
%REQUIREIDLEPOOL Не отменяет чужие задачи и не измеряет при их параллельной нагрузке.
    if ~isempty(pool.FevalQueue.RunningFutures) || ~isempty(pool.FevalQueue.QueuedFutures)
        error('comparison:PoolBusy', 'Завершите другие задачи пула перед сравнением времени.');
    end
end

function [record, stored] = finishTrial(record, planned, scenario)
%FINISHTRIAL Разделяет допустимость геометрического пути и minimum-snap.
    record.CalculationTime_s = planned.calculationTime;
    stored = struct('rawPath', planned.path, 'pp', [], 'validation', []);
    if isempty(planned.path)
        record.Status = "no_path";
        record.Message = "Геометрический маршрут не найден.";
        if isfield(planned.tree, 'message'), record.Message = string(planned.tree.message); end
        return;
    end
    validationStart = tic;
    rawFree = norm(planned.path(1, :)-scenario.start2d) < 1e-8 && ...
        norm(planned.path(end, :)-scenario.goal2d) < 1e-8;
    for index = 1:max(1, size(planned.path, 1)-1)
        next = min(index+1, size(planned.path, 1));
        rawFree = rawFree && voronoi.isSegmentFree(planned.path(index, :), ...
            planned.path(next, :), scenario.map, scenario.obstacles, scenario.safetyRadius);
    end
    record.ValidationTime_s = toc(validationStart);
    record.RawPathLength_m = sum(vecnorm(diff(planned.path, 1, 1), 2, 2));
    if ~rawFree
        record.Status = "invalid_path";
        record.Message = "Недопустимый исходный геометрический путь.";
        return;
    end
    record.PlanningSolved = true;
    rng(planned.rngState);
    postStart = tic;
    try
        experiment = runPlannerExperiment(scenario, planned.plannerName, false, planned);
        record.PostprocessingTime_s = toc(postStart);
        stored.pp = experiment.referenceTrajectory.pp;
        validationStart = tic;
        [free, stored.validation] = geometry.isPolynomialTrajectoryFree(stored.pp, ...
            scenario.map, scenario.obstacles, scenario.safetyRadius);
        record.ValidationTime_s = record.ValidationTime_s+toc(validationStart);
        record.PipelineTime_s = record.CalculationTime_s+record.PostprocessingTime_s+record.ValidationTime_s;
        if ~free
            record.Status = "invalid_trajectory";
            record.Message = string(stored.validation.message);
            return;
        end
        record.Status = "solved";
        record.Solved = true;
        record.PathLength_m = experiment.metrics.pathLength;
        record.MaxCurvature_1_m = experiment.metrics.maxCurvature;
        record.MeanCurvature_1_m = experiment.metrics.meanCurvature;
    catch exception
        record.Status = "error";
        record.Message = string(exception.message);
        record.ErrorIdentifier = string(exception.identifier);
    end
end
