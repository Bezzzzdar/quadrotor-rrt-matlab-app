%RUN_ANALYSIS Сравнивает методы и исследует их параметры на общих случайных картах.

clear;
clc;
close all;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));
runMode = "plots"; % "experiment" - расчёт, "plots" - только сохранённые графики.
resultDir = fullfile(rootDir, 'results', 'analysis');
if runMode == "plots"
    [figures, parameterStudy] = viz.plotSavedParameterStudy(resultDir);
    return;
elseif runMode ~= "experiment"
    error('analysis:InvalidRunMode', 'runMode должен быть experiment или plots.');
end
baseScenario = config.defaultScenario();
configurations = analysis.baselineConfigurations();

options.sceneCount = 10;
options.plannerSeedBase = 1001;
options.sceneSeedBase = 200001;
options.useParallel = true;
options.maxWorkers = 3;
options.showProgress = true;
options.obstacles = struct('countRange', [5, 20], 'minSize_m', [40, 40], ...
    'maxSize_m', [180, 180], 'maxAreaFraction', 0.3, 'maxPlacementAttempts', 10000);

% Меняется один параметр; остальные настройки метода сохраняют базовые значения.
numValues = 10;
variants.rrt.stepSize = struct('value', linspace(10, 100, numValues), 'label', 'D, м');
variants.rrt.goalBias = struct('value', linspace(0.05, 0.5, numValues), 'label', 'W_{ц}');
variants.rrt.maxIter = struct('value', round(linspace(1000, 10000, numValues)), 'label', 'I_{макс}');
variants.astar.heuristicType = struct('value', ["euclidean", "octile"], ...
    'label', 'H(n)', 'tickLabels', {{'Евклидова', 'Октильная'}});
variants.astar.heuristicWeight = struct('value', linspace(0.25, 1, numValues), 'label', 'w');
variants.astar.tieBreaking = struct('value', ["largerg", "smallerg"], ...
    'label', 'K(n)', 'tickLabels', {{'Больший g', 'Меньший g'}});
variants.voronoi.clearanceWeight = struct('value', logspace(0, 2, numValues), ...
    'label', '\lambda', 'scale', 'log');
variants.voronoi.clearanceExponent = struct('value', linspace(0.5, 2, numValues), 'label', 'p');
variants.voronoi.connectionCount = struct('value', 1:numValues, 'label', 'K');

analysisResult = analysis.runExperiment(baseScenario, configurations, options);

if ~isfolder(resultDir), mkdir(resultDir); end
save(fullfile(resultDir, 'analysis_results.mat'), 'analysisResult');
workbook = fullfile(resultDir, 'analysis_summary.xlsx');
for index = 1:numel(configurations)
    planner = char(configurations(index).Planner);
    methodTable = analysisResult.tables.(planner);
    probability = analysisResult.probabilities.(planner);
    fprintf('\n%s\n', configurations(index).Name);
    disp(methodTable);
    fprintf('Вероятность решения задачи: %.6g (%.1f%%)\n', probability, 100*probability);

    mode = 'overwritesheet';
    if index == 1, mode = 'replacefile'; end
    writetable(analysis.excelSafeTable(methodTable), workbook, ...
        'Sheet', planner, 'Range', 'A3', 'WriteMode', mode, 'UseExcel', false);
    writecell({configurations(index).Name}, workbook, 'Sheet', planner, ...
        'Range', 'A1', 'WriteMode', 'inplace', 'UseExcel', false);
    writecell({'Вероятность решения задачи', probability}, workbook, ...
        'Sheet', planner, 'Range', 'A9', 'WriteMode', 'inplace', 'UseExcel', false);
    writetable(methodTable, fullfile(resultDir, ['analysis_', planner, '.csv']));
end

% Карты создаются один раз и повторно используются во всех точках исследования.
parameterStudy = analysis.runParameterStudy(baseScenario, configurations, variants, ...
    options, analysisResult.scenes);
% Сохраняем длительный расчёт до экспорта: ошибка записи Excel не теряет результаты.
save(fullfile(resultDir, 'parameter_study.mat'), 'parameterStudy');
analysis.exportParameterStudy(parameterStudy, fullfile(resultDir, 'parameters'));
figures = viz.plotSavedParameterStudy(resultDir);
