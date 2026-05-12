% RUN_ANALYSIS Параметрический анализ RRT

clear;
clc;
close all;

%% Init
if isempty(gcp('nocreate'))
    parpool;
end


rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));

baseScenario = config.defaultScenario();

%% Parameter ranges
% Выберите 4 исследуемых параметра

mapSize = norm(baseScenario.map.height);

rrtVariants.stepSize = linspace(10, 100, 10);
rrtVariants.goalBias = linspace(0.1, 1, 10);
rrtVariants.maxIter = linspace(1000, 10000, 10);
rrtVariants.goalThreshold = linspace(10, 100, 10);

%% Run analysis

results = runRRTAnalysis(baseScenario, rrtVariants);

%% Print summary tables

disp('======================================================');
disp('STEP SIZE');
disp(results.stepSize);

disp('======================================================');
disp('GOAL BIAS');
disp(results.goalBias);

disp('======================================================');
disp('MAX ITER');
disp(results.maxIter);

disp('======================================================');
disp('GOAL THRESHOLD');
disp(results.goalThreshold);

%% Plot graphs

plotRRTAnalysis(results);

%% =======================================================================
%% FUNCTIONS
%% =======================================================================

function results = runRRTAnalysis(baseScenario, variants)
%RUNRRTANALYSIS Выполняет параметрический анализ RRT.
%
% Для каждого значения параметра:
%   - выполняется 100 запусков,
%   - вычисляются средние значения,
%   - вычисляются стандартные отклонения.
%
% Используется parallel computing.

    runsPerValue = 100;

    variantNames = fieldnames(variants);

    results = struct();

    for v = 1:numel(variantNames)

        paramName = variantNames{v};
        paramValues = variants.(paramName);

        fprintf("\n==================================================\n");
        fprintf("Parameter: %s\n", paramName);
        fprintf("==================================================\n");

        resultTable = table();

        for i = 1:numel(paramValues)

            value = paramValues(i);

            fprintf( ...
                "[%d/%d] %s = %.3f\n", ...
                i, numel(paramValues), paramName, value);

            calcTimeValues = nan(runsPerValue, 1);
            pathLengthValues = nan(runsPerValue, 1);
            maxCurvatureValues = nan(runsPerValue, 1);
            meanCurvatureValues = nan(runsPerValue, 1);

            %% Parallel experiments

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

            %% Aggregate statistics

            row = table( ...
                value, ...
                mean(calcTimeValues, 'omitnan'), ...
                std(calcTimeValues, 'omitnan'), ...
                mean(pathLengthValues, 'omitnan'), ...
                std(pathLengthValues, 'omitnan'), ...
                mean(maxCurvatureValues, 'omitnan'), ...
                std(maxCurvatureValues, 'omitnan'), ...
                mean(meanCurvatureValues, 'omitnan'), ...
                std(meanCurvatureValues, 'omitnan'), ...
                'VariableNames', { ...
                'ParameterValue', ...
                'MeanCalculationTime_s', ...
                'StdCalculationTime_s', ...
                'MeanPathLength_m', ...
                'StdPathLength_m', ...
                'MeanMaxCurvature_1_m', ...
                'StdMaxCurvature_1_m', ...
                'MeanMeanCurvature_1_m', ...
                'StdMeanCurvature_1_m'});

            resultTable = [resultTable; row];

        end

        results.(paramName) = resultTable;

    end

end

%% =======================================================================

function plotRRTAnalysis(results)
%PLOTRRTANALYSIS Строит графики анализа параметров RRT.

    resultNames = fieldnames(results);

    for i = 1:numel(resultNames)

        paramName = resultNames{i};

        tbl = results.(paramName);

        x = tbl.ParameterValue;

        figure( ...
            'Name', sprintf('Analysis: %s', paramName), ...
            'NumberTitle', 'off');

        sgtitle( ...
            sprintf('Influence of parameter "%s"', paramName), ...
            'Interpreter', 'none');

        %% Calculation time

        subplot(2,2,1);

        errorbar( ...
            x, ...
            tbl.MeanCalculationTime_s, ...
            tbl.StdCalculationTime_s, ...
            '-', ...
            'LineWidth', 1.5);

        grid on;
        grid minor;

        xlabel(paramName);
        ylabel('t, s');

        title('Calculation time');

        %% Path length

        subplot(2,2,2);

        errorbar( ...
            x, ...
            tbl.MeanPathLength_m, ...
            tbl.StdPathLength_m, ...
            '-', ...
            'LineWidth', 1.5);

        grid on;
        grid minor;

        xlabel(paramName);
        ylabel('L, m');

        title('Path length');

        %% Maximum curvature

        subplot(2,2,3);

        errorbar( ...
            x, ...
            tbl.MeanMaxCurvature_1_m, ...
            tbl.StdMaxCurvature_1_m, ...
            '-', ...
            'LineWidth', 1.5);

        grid on;
        grid minor;

        xlabel(paramName);
        ylabel('C, 1/m');

        title('Maximum curvature');

        %% Mean curvature

        subplot(2,2,4);

        errorbar( ...
            x, ...
            tbl.MeanMeanCurvature_1_m, ...
            tbl.StdMeanCurvature_1_m, ...
            '-', ...
            'LineWidth', 1.5);

        grid on;
        grid minor;

        xlabel(paramName);
        ylabel('C, 1/m');

        title('Mean curvature');

        set(findall(gcf,'-property','FontName'), ...
            'FontName', 'Times');

        set(findall(gcf,'-property','FontSize'), ...
            'FontSize', 14);

    end

end