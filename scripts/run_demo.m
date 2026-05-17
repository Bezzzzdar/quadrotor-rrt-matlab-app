%RUN_DEMO Пример запуска без графического интерфейса.

clear;
clc;
close all;
rng(121314)

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));

scenario = config.defaultScenario();
result = runSingleExperiment(scenario);

figure('Name', 'Демонстрационный запуск');
viz.plotScenario(gca, scenario.map, scenario.obstacles, scenario.start2d, scenario.goal2d, result.tree, result.path, result.simLog, result.rawPath, result.rrtPath);

if ~isempty(result.simLog)
    viz.plotTrajectoryDerivatives(result.simLog);
end
