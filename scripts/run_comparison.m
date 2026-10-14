%RUN_COMPARISON Сравнение по протоколу Moll, Sucan, Kavraki (2015).
% https://www.kavrakilab.org/publications/moll-sucan2015benchmarking-motion-planning.pdf

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));
baseScenario = config.defaultScenario();

% Это настройки эксперимента, а не рекомендованные статьёй числовые значения.
options.runCount = 10;
options.seedBase = 1001;
options.timeLimit_s = 30;

% Дополнительные фиксированные конфигурации можно добавить отдельными строками.
% Здесь нет автоматического выбора «оптимальных» настроек из однопараметрических серий.
configurations = struct('Name', {'RRT', 'A*', 'Вороной'}, ...
    'Planner', {'rrt', 'astar', 'voronoi'}, ...
    'Parameters', {baseScenario.rrt, baseScenario.astar, baseScenario.voronoi});

comparison = analysis.runComparison(baseScenario, configurations, options);
disp(comparison.summary);
disp(comparison.statistics);

resultDir = fullfile(rootDir, 'results');
analysis.exportComparison(comparison, resultDir);
criteriaFigure = viz.plotPlannerComparison(comparison);
reliabilityFigure = viz.plotComparisonReliability(comparison);
exportgraphics(criteriaFigure, fullfile(resultDir, 'comparison_criteria.png'));
exportgraphics(criteriaFigure, fullfile(resultDir, 'comparison_criteria.pdf'), 'ContentType', 'vector');
exportgraphics(reliabilityFigure, fullfile(resultDir, 'comparison_reliability.png'));
exportgraphics(reliabilityFigure, fullfile(resultDir, 'comparison_reliability.pdf'), 'ContentType', 'vector');
