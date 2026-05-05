%RUN_DEMO Пример запуска без графического интерфейса.

clear;
clc;
close all;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));

scenario = config.defaultScenario();
result = runSingleExperiment(scenario);

disp(result.summary);

figure('Name', 'Демонстрационный запуск');
viz.plotScenario(gca, scenario.map, scenario.obstacles, scenario.start2d, scenario.goal2d, result.tree, result.path, result.simLog, result.rawPath, result.rrtPath);

if ~isempty(result.simLog)
    figure('Name', 'Ошибка слежения');
    plot(result.simLog.t, result.simLog.trackingError, 'LineWidth', 2);
    grid on;
    xlabel('Время, с');
    ylabel('Ошибка, м');
    title('Ошибка слежения динамической модели');
end
