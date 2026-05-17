% RUN_ANALYSIS Параметрический анализ RRT

clear;
clc;
close all;

%% Инициализация
if isempty(gcp('nocreate'))
    parpool;
end

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));

baseScenario = config.defaultScenario();

%% Параметры

numVariants = 10;

rrtVariants.stepSize.value = linspace(10, 100, numVariants);
rrtVariants.stepSize.label = 'D, м';
rrtVariants.goalBias.value = linspace(0.1, 0.5, numVariants);
rrtVariants.goalBias.label = 'W_ц';
rrtVariants.maxIter.value = linspace(1000, 10000, numVariants);
rrtVariants.maxIter.label = 'I';
rrtVariants.goalThreshold.value = linspace(1, 10, numVariants);
rrtVariants.goalThreshold.label = 'R_д';

%% Запуск сравнения

results = runRRTAnalysis(baseScenario, rrtVariants);

%% Таблицы с результатами

disp('======================================================');
disp('STEP SIZE');
disp(results.stepSize);
writetable(results.stepSize, 'tableStepSize.xlsx');

disp('======================================================');
disp('GOAL BIAS');
disp(results.goalBias);
writetable(results.goalBias, 'tableGoalBias.xlsx');

disp('======================================================');
disp('MAX ITER');
disp(results.maxIter);
writetable(results.maxIter, 'tableMaxIter.xlsx');

disp('======================================================');
disp('GOAL THRESHOLD');
disp(results.goalThreshold);
writetable(results.goalThreshold, 'tableGoalThreshold.xlsx');

% Графики

plotRRTAnalysis(results, rrtVariants);

%% =======================================================================
%% Вспомогательные функции
%% =======================================================================

function results = runRRTAnalysis(baseScenario, variants)
%RUNRRTANALYSIS Выполняет параметрический анализ RRT.

    runsPerValue = 10;

    variantNames = fieldnames(variants);

    results = struct();

    for v = 1:numel(variantNames)

        paramName = variantNames{v};
        paramValues = variants.(paramName).value;

        fprintf("\n==================================================\n");
        fprintf("Parameter: %s\n", paramName);
        fprintf("==================================================\n");

        resultTable = table();

        for i = 1:numel(paramValues)

            value = paramValues(i);

            fprintf("[%d/%d] %s = %.3f\n", i, numel(paramValues), paramName, value);

            calcTimeValues = nan(runsPerValue, 1);
            pathLengthValues = nan(runsPerValue, 1);
            maxCurvatureValues = nan(runsPerValue, 1);
            meanCurvatureValues = nan(runsPerValue, 1);

            %% Паралльельные эксперименты

            parfor k = 1:runsPerValue
                rng('shuffle');
                scenario = baseScenario;

                % Меняем только один параметр
                scenario.rrt.(paramName) = value;

                try

                    result = runSingleExperiment(scenario);

                    metrics = result.metrics;

                    calcTimeValues(k) = result.calculationTime;
                    pathLengthValues(k) = metrics.pathLength;
                    maxCurvatureValues(k) = metrics.maxCurvature;
                    meanCurvatureValues(k) = metrics.meanCurvature;

                catch

                    % parfor плохо работает с warning/spam output
                    % просто оставляем NaN

                end

            end

            %% Сбор статистик

            row = table( ...
                value, ...
                mean(calcTimeValues, 'omitnan'), ...
                mean(pathLengthValues, 'omitnan'), ...
                mean(maxCurvatureValues, 'omitnan'), ...
                mean(meanCurvatureValues, 'omitnan'), ...
                'VariableNames', { ...
                'ParameterValue', ...
                'MeanCalculationTime_s', ...
                'MeanPathLength_m', ...
                'MeanMaxCurvature_1_m', ...
                'MeanMeanCurvature_1_m'});

            resultTable = [resultTable; row]; %#ok<AGROW>

        end

        results.(paramName) = resultTable;

    end

end

function plotRRTAnalysis(results, variants)
%PLOTRRTANALYSIS Строит графики анализа параметров RRT.

    paramNames = fieldnames(results);
    metrics = {
        'MeanCalculationTime_s', 't, с', 'Время вычисления';
        'MeanPathLength_m', 'L, м', 'Длина пути';
        'MeanMaxCurvature_1_m', 'k_{макс}, 1/м', 'Максимальная кривизна траектории';
        'MeanMeanCurvature_1_m', 'k_{сред}, 1/м', 'Средняя кривизна траектории'
    };
    colors = lines(numel(paramNames));

    for metricIndex = 1:size(metrics, 1)
        figure('Name', metrics{metricIndex, 3}, 'NumberTitle', 'off');

        mainPosition = [0.14, 0.36, 0.72, 0.48];
        dataAx = axes('Position', mainPosition);
        hold(dataAx, 'on');
        grid(dataAx, 'on');
        grid(dataAx, 'minor');
        xlim(dataAx, [0, 1]);
        ylabel(dataAx, metrics{metricIndex, 2});
        dataAx.XTick = [];

        legendEntries = cell(numel(paramNames), 1);

        for paramIndex = 1:numel(paramNames)
            paramName = paramNames{paramIndex};
            tbl = results.(paramName);
            x = normalizeParameter(tbl.ParameterValue);
            y = tbl.(metrics{metricIndex, 1});

            plot(dataAx, x, y, '-', ...
                'Color', colors(paramIndex, :), ...
                'LineWidth', 1.5);

            legendEntries{paramIndex} = variants.(paramName).label;
        end

        legend(dataAx, legendEntries, 'Location', 'northeast');
        addParameterAxes(results, variants, paramNames, mainPosition);

        set(findall(gcf,'-property','FontName'), 'FontName', 'Times');
        set(findall(gcf,'-property','FontSize'), 'FontSize', 14);
        set(findall(gcf,'-property','Interpreter'), 'Interpreter', 'tex');

    end

end

function xNorm = normalizeParameter(x)
%NORMALIZEPARAMETER Переводит значения параметра в общую координату [0, 1].

    xMin = min(x);
    xMax = max(x);

    if abs(xMax - xMin) < eps
        xNorm = zeros(size(x));
        return;
    end

    xNorm = (x - xMin) / (xMax - xMin);
end

function addParameterAxes(results, variants, paramNames, mainPosition)
%ADDPARAMETERAXES Добавляет четыре независимые шкалы X к одному графику.

    axisLocations = repmat({'bottom'}, 1, numel(paramNames));
    verticalOffsets = -0.08 * (0:numel(paramNames) - 1);

    for paramIndex = 1:numel(paramNames)
        paramName = paramNames{paramIndex};
        tbl = results.(paramName);
        xValues = tbl.ParameterValue;

        axisPosition = mainPosition;
        axisPosition(2) = axisPosition(2) + verticalOffsets(paramIndex);

        ax = axes('Position', axisPosition, ...
            'Color', 'none', ...
            'YAxisLocation', 'right', ...
            'YTick', [], ...
            'YColor', 'none', ...
            'XAxisLocation', axisLocations{paramIndex}, ...
            'Box', 'off');

        ax.XLim = [min(xValues), max(xValues)];
        ax.XTick = linspace(min(xValues), max(xValues), 5);
        xlabel(ax, variants.(paramName).label);
    end

end
