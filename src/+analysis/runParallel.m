function results = runParallel(baseScenario, variants, runsPerValue, seedBase)
%RUNPARALLEL Выполняет три независимых цикла анализа на трёх работниках одновременно.
% Параллельный запуск: https://www.mathworks.com/help/parallel-computing/parfeval.html
    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool('Processes', 3);
    elseif pool.NumWorkers < 3
        error('analysis:InsufficientWorkers', ...
            'Для одновременного анализа трёх методов нужен пул минимум из трёх работников.');
    end
    progress = parallel.pool.DataQueue;
    afterEach(progress, @printProgress);
    methods = {'rrt', 'astar', 'voronoi'};
    futures(1:3) = parallel.FevalFuture;
    for index = 1:3
        method = methods{index};
        futures(index) = parfeval(pool, @analysis.runMethod, 1, ...
            baseScenario, method, variants.(method), runsPerValue, seedBase, progress);
    end
    cleanup = onCleanup(@() cancel(futures));
    results = struct();
    for index = 1:3
        [completed, value] = fetchNext(futures);
        results.(methods{completed}) = value;
    end
end

function printProgress(data)
%PRINTPROGRESS Выводит прогресс завершённых значений без дополнительных окон.
    fprintf('[%s] %s=%s (%d/%d): найдено %d/%d\n', ...
        data.planner, data.parameter, data.value, data.index, data.count, data.found, data.runs);
end
