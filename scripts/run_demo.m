%RUN_DEMO Пример запуска без графического интерфейса.

clear;
clc;
close all;
rng(121314)

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));

scenario = config.defaultScenario();

rrtResult = runPlannerExperiment(scenario, 'rrt');
astarResult = runPlannerExperiment(scenario, 'astar');
voronoiResult = runPlannerExperiment(scenario, 'voronoi');

viz.plotPlannerResult(scenario, rrtResult);
viz.plotPlannerResult(scenario, astarResult);
viz.plotPlannerResult(scenario, voronoiResult);

comparison = [rrtResult.summary; astarResult.summary; voronoiResult.summary];
comparison = addvars(comparison, ["RRT"; "A*"; "GVD Voronoi"], ...
    'Before', 1, 'NewVariableNames', 'Planner');

disp('======================================================');
disp('Сравнение методов планирования');
disp(comparison);
