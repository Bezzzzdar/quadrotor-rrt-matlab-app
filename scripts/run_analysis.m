%RUN_ANALYSIS Параллельный параметрический анализ RRT, A* и Вороного.

clear;
clc;
close all;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));
baseScenario = config.defaultScenario();

numVariants = 10;
runsPerValue = 10;
seedBase = 42;

% Меняются только параметры метода; карта и параметры траектории неизменны.
variants.rrt.stepSize = struct('value', linspace(10, 100, numVariants), 'label', 'Шаг D, м');
variants.rrt.goalBias = struct('value', linspace(0.1, 0.5, numVariants), 'label', 'Вероятность цели W');
variants.rrt.maxIter = struct('value', round(linspace(1000, 10000, numVariants)), 'label', 'Лимит итераций I');

variants.astar.heuristicType = struct('value', ["euclidean", "octile"], ...
    'label', 'Тип эвристики', 'tickLabels', {{'Евклидова', 'Октильная'}});
variants.astar.heuristicWeight = struct('value', linspace(0.25, 1, numVariants), 'label', 'Вес эвристики w');
variants.astar.tieBreaking = struct('value', ["largerg", "smallerg"], ...
    'label', 'Равные приоритеты', 'tickLabels', {{'Больший g', 'Меньший g'}});

variants.voronoi.clearanceWeight = struct('value', logspace(0, 2, numVariants), ...
    'label', 'Вес отступа \lambda (лог.)', 'scale', 'log');
variants.voronoi.clearanceExponent = struct('value', linspace(0.5, 2, numVariants), 'label', 'Степень штрафа p');
variants.voronoi.connectionCount = struct('value', 1:numVariants, 'label', 'Число подключений K');

% Три parfeval-задачи запускают три цикла анализа одновременно, без вложенных parfor.
results = analysis.runParallel(baseScenario, variants, runsPerValue, seedBase);

methods = {'rrt', 'astar', 'voronoi'};
displayNames = {'RRT', 'A*', 'Вороной'};
for methodIndex = 1:3
    method = methods{methodIndex};
    disp('======================================================');
    disp(displayNames{methodIndex});
    names = fieldnames(variants.(method));
    for parameter = 1:numel(names)
        disp(variants.(method).(names{parameter}).label);
        disp(results.(method).tables.(names{parameter}));
    end
    viz.plotPlannerAnalysis(results.(method), variants.(method), displayNames{methodIndex});
end

resultDir = fullfile(rootDir, 'results');
if ~isfolder(resultDir), mkdir(resultDir); end
save(fullfile(resultDir, 'planner_analysis.mat'), ...
    'baseScenario', 'variants', 'runsPerValue', 'seedBase', 'results');

% Итоговые таблицы каждого метода сохраняются в отдельную книгу Excel.
analysis.exportAnalysis(results, variants, resultDir);
