%RUN_ANALYSIS Сравнивает фиксированные методы на десяти общих случайных картах.

clear;
clc;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));
baseScenario = config.defaultScenario();
configurations = analysis.baselineConfigurations();

options.sceneCount = 10;
options.plannerSeedBase = 1001;
options.sceneSeedBase = 200001;
options.obstacles = struct('countRange', [5, 20], 'minSize_m', [40, 40], ...
    'maxSize_m', [180, 180], 'maxAreaFraction', 0.3, 'maxPlacementAttempts', 10000);

analysisResult = analysis.runExperiment(baseScenario, configurations, options);

resultDir = fullfile(rootDir, 'results', 'analysis');
if ~isfolder(resultDir), mkdir(resultDir); end
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
save(fullfile(resultDir, 'analysis_results.mat'), 'analysisResult');
