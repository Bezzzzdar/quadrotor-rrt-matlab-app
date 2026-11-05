function tests = testParameterStudy
%TESTPARAMETERSTUDY Проверяет пофакторный анализ, его графики и сохранение данных.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Выполняет небольшое исследование всех девяти параметров на общих картах.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.oldRng = rng;
    addpath(genpath(fullfile(root, 'src')));
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
    scenario = config.defaultScenario();
    scenario.map = struct('width', 40, 'height', 40, 'resolution', 1);
    scenario.start2d = [3, 3];
    scenario.goal2d = [37, 37];
    scenario.safetyRadius = 1;
    scenario.sim.vRef = 5;
    scenario.sim.dt = 0.1;
    configurations = analysis.baselineConfigurations();
    configurations(1).Parameters.stepSize = 5;
    configurations(1).Parameters.goalBias = 0.3;
    configurations(1).Parameters.maxIter = 1000;
    configurations(1).Parameters.smoothingIterations = 20;
    variants.rrt.stepSize = struct('value', [4, 8], 'label', 'Шаг дерева, м');
    variants.rrt.goalBias = struct('value', [0.1, 0.4], 'label', 'Вероятность выборки цели');
    variants.rrt.maxIter = struct('value', [300, 600], 'label', 'Лимит итераций');
    variants.astar.heuristicType = struct('value', ["euclidean", "octile"], ...
        'label', 'Тип эвристики', 'tickLabels', {{'Евклидова', 'Октильная'}});
    variants.astar.heuristicWeight = struct('value', [0.5, 1], 'label', 'Вес эвристики');
    variants.astar.tieBreaking = struct('value', ["largerg", "smallerg"], ...
        'label', 'Правило равных приоритетов');
    variants.voronoi.clearanceWeight = struct('value', [9, 30], 'label', 'Вес штрафа', 'scale', 'log');
    variants.voronoi.clearanceExponent = struct('value', [1, 1.5], 'label', 'Степень штрафа');
    variants.voronoi.connectionCount = struct('value', [1, 3], 'label', 'Число подключений');
    options = struct('sceneCount', 3, 'plannerSeedBase', 1001, 'sceneSeedBase', 6001, ...
        'obstacles', struct('countRange', [1, 3], 'minSize_m', [3, 3], ...
        'maxSize_m', [7, 7], 'maxAreaFraction', 0.3, 'maxPlacementAttempts', 1000));
    figures = findall(groot, 'Type', 'figure');
    study = analysis.runParameterStudy(scenario, configurations, variants, options);
    testCase.TestData.rngAfterStudy = rng;
    testCase.TestData.figuresUnchanged = isequal(findall(groot, 'Type', 'figure'), figures);
    testCase.TestData.study = study;
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает окружение MATLAB.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.oldRng);
end

function testCommonScenesAndSeeds(testCase)
%TESTCOMMONSCENESANDSEEDS Проверяет общие препятствия, seed и сохранение окружения.
    study = testCase.TestData.study;
    verifyEqual(testCase, testCase.TestData.rngAfterStudy, testCase.TestData.oldRng);
    verifyTrue(testCase, testCase.TestData.figuresUnchanged);
    verifyEqual(testCase, study.scenes, analysis.generateScenes(study.scenario, study.options));
    for planner = {'rrt', 'astar', 'voronoi'}
        parameters = fieldnames(study.methods.(planner{1}));
        verifyNumElements(testCase, parameters, 3);
        for index = 1:numel(parameters)
            samples = study.methods.(planner{1}).(parameters{index}).samples;
            verifyEqual(testCase, samples.SceneId, repmat((1:3)', 2, 1));
            verifyEqual(testCase, samples.SceneSeed, repmat((6001:6003)', 2, 1));
            verifyEqual(testCase, samples.PlannerSeed, repmat((1001:1003)', 2, 1));
            verifyEqual(testCase, samples.ValueIndex, repelem((1:2)', 3));
            verifyFalse(testCase, any(samples.Status == "error"));
        end
    end
end

function testOnlyOneParameterChanges(testCase)
%TESTONLYONEPARAMETERCHANGES Сохраняет исходные данные и прочие параметры метода.
    study = testCase.TestData.study;
    for baseline = study.configurations
        planner = char(baseline.Planner);
        parameters = fieldnames(study.methods.(planner));
        for index = 1:numel(parameters)
            parameter = parameters{index};
            actual = study.methods.(planner).(parameter);
            values = study.variants.(planner).(parameter).value;
            for valueIndex = 1:numel(values)
                expected = baseline;
                expected.Parameters.(parameter) = values(valueIndex);
                verifyEqual(testCase, actual.configurations(valueIndex), expected);
                samples = actual.samples(actual.samples.ValueIndex == valueIndex, :);
                verifyEqual(testCase, samples.ParameterValue, repmat(values(valueIndex), 3, 1));
            end
        end
    end
end

function testConditionalStatistics(testCase)
%TESTCONDITIONALSTATISTICS Независимо пересчитывает условные моменты и вероятность.
    study = testCase.TestData.study;
    metrics = {'PlanningTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'};
    for planner = {'rrt', 'astar', 'voronoi'}
        parameters = fieldnames(study.methods.(planner{1}));
        for index = 1:numel(parameters)
            actual = study.methods.(planner{1}).(parameters{index});
            verifySize(testCase, actual.table, [2, 12]);
            for valueIndex = 1:height(actual.table)
                samples = actual.samples(actual.samples.ValueIndex == valueIndex, :);
                verifyEqual(testCase, actual.table.AttemptCount(valueIndex), 3);
                verifyEqual(testCase, actual.table.SuccessCount(valueIndex), sum(samples.Solved));
                verifyEqual(testCase, actual.table.Probability(valueIndex), sum(samples.Solved)/3);
                verifyEqual(testCase, samples.PlanningTime_s, ...
                    samples.AlgorithmTime_s + samples.MinimumSnapTime_s, 'AbsTol', 1e-12);
                for metric = 1:numel(metrics)
                    values = samples.(metrics{metric});
                    values = values(samples.Solved & ~isnan(values));
                    expectation = NaN;
                    variance = NaN;
                    if ~isempty(values), expectation = mean(values); end
                    if numel(values) >= 2 && all(isfinite(values)), variance = var(values, 0); end
                    verifyEqual(testCase, actual.table.(['Mean', metrics{metric}])(valueIndex), expectation);
                    verifyEqual(testCase, actual.table.(['Variance', metrics{metric}])(valueIndex), variance);
                end
            end
        end
    end
end

function testFailureIsNotAZeroCriterion(testCase)
%TESTFAILUREISNOTAZEROCRITERION Сохраняет неудачные карты в знаменателе вероятности.
    study = testCase.TestData.study;
    configuration = study.configurations(1);
    configuration.Parameters.maxIter = 1;
    configuration.Parameters.stepSize = 1;
    variants.rrt.goalBias = study.variants.rrt.goalBias;
    actual = analysis.runParameterStudy(study.scenario, configuration, variants, study.options, study.scenes);
    summary = actual.methods.rrt.goalBias.table;
    verifyEqual(testCase, summary.Probability, [0; 0]);
    verifyEqual(testCase, summary.AttemptCount, [3; 3]);
    verifyEqual(testCase, summary.SuccessCount, [0; 0]);
    verifyTrue(testCase, all(isnan(summary{:, 5:end}), 'all'));
end

function testInvalidInputs(testCase)
%TESTINVALIDINPUTS Не допускает повторяющиеся значения и подмену общих карт.
    study = testCase.TestData.study;
    variants.rrt.stepSize = struct('value', [5, 5], 'label', 'Шаг');
    verifyError(testCase, @() analysis.runParameterStudy(study.scenario, ...
        study.configurations(1), variants, study.options, study.scenes), 'analysis:InvalidParameterValues');
    variants.rrt.unknown = variants.rrt.stepSize;
    variants.rrt = rmfield(variants.rrt, 'stepSize');
    verifyError(testCase, @() analysis.runParameterStudy(study.scenario, ...
        study.configurations(1), variants, study.options, study.scenes), 'analysis:InvalidParameter');
    scenes = study.scenes;
    scenes(1).seed = 1;
    verifyError(testCase, @() analysis.runExperiment(study.scenario, ...
        study.configurations, study.options, scenes), 'analysis:InvalidScenes');
    verifyError(testCase, @() analysis.runExperiment(study.scenario, ...
        study.configurations, study.options, scenes(1)), 'analysis:InvalidScenes');
end

function testExport(testCase)
%TESTEXPORT Проверяет книги с тремя листами и CSV всех значений и попыток.
    study = testCase.TestData.study;
    folder = tempname;
    mkdir(folder);
    cleanup = onCleanup(@() rmdir(folder, 's'));
    analysis.exportParameterStudy(study, folder);
    analysis.exportParameterStudy(study, folder);
    for configuration = study.configurations
        planner = char(configuration.Planner);
        parameters = fieldnames(study.methods.(planner));
        workbook = fullfile(folder, ['parameters_', planner, '.xlsx']);
        verifyEqual(testCase, sheetnames(workbook), string(parameters));
        for index = 1:numel(parameters)
            parameter = parameters{index};
            expected = study.methods.(planner).(parameter).table;
            saved = readtable(workbook, 'Sheet', parameter, 'Range', 'A3');
            verifyEqual(testCase, height(saved), 2);
            verifyEqual(testCase, string(saved.ParameterValue), string(expected.ParameterValue));
            verifyEqual(testCase, saved.Probability, expected.Probability);
            heading = readcell(workbook, 'Sheet', parameter, 'Range', 'A1:B1');
            verifyEqual(testCase, string(heading), [string(configuration.Name), ...
                string(study.variants.(planner).(parameter).label)]);
            runs = readtable(fullfile(folder, [planner, '_', parameter, '_runs.csv']));
            verifyEqual(testCase, height(runs), 6);
            verifyTrue(testCase, isfile(fullfile(folder, [planner, '_', parameter, '.csv'])));
        end
    end
end

function testCharts(testCase)
%TESTCHARTS Проверяет пять вкладок, пары полей и три строго совмещённые шкалы X.
    study = testCase.TestData.study;
    visible = get(groot, 'DefaultFigureVisible');
    set(groot, 'DefaultFigureVisible', 'off');
    visibilityCleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', visible));
    for configuration = study.configurations
        planner = char(configuration.Planner);
        fig = viz.plotParameterStudy(study, planner);
        figureCleanup = onCleanup(@() close(fig));
        verifyEqual(testCase, fig.Name, configuration.Name);
        verifyNumElements(testCase, findall(fig, 'Type', 'uitab'), 5);
        verifyEmpty(testCase, findobj(fig, 'Tag', 'StudyStatisticSelector'));
        group = findobj(fig, 'Tag', 'StudyTabs');
        metrics = {'PlanningTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'};
        for metric = 1:numel(metrics)
            tab = findobj(group, 'Tag', metrics{metric});
            group.SelectedTab = tab;
            feval(group.SelectionChangedFcn, group, []);
            verifyNumElements(testCase, findall(tab, '-regexp', 'Tag', '^StudyField_'), 2);
            meanField = findobj(tab, 'Tag', ['StudyField_Mean', metrics{metric}]);
            varianceField = findobj(tab, 'Tag', ['StudyField_Variance', metrics{metric}]);
            verifyLessThan(testCase, meanField.Position(1), varianceField.Position(1));
            verifyField(testCase, meanField, study, planner, ['Mean', metrics{metric}]);
            verifyField(testCase, varianceField, study, planner, ['Variance', metrics{metric}]);
        end
        group.SelectedTab = findobj(group, 'Tag', 'Probability');
        feval(group.SelectionChangedFcn, group, []);
        verifyNumElements(testCase, findall(group.SelectedTab, '-regexp', 'Tag', '^StudyField_'), 1);
        field = findobj(group.SelectedTab, 'Tag', 'StudyField_Probability');
        verifyField(testCase, field, study, planner, 'Probability');
        ax = findall(field, 'Tag', 'StudyDataAxes');
        verifyEqual(testCase, ax.YLim, [0, 1]);
        axesHandles = findall(fig, 'Type', 'axes');
        verifyNumElements(testCase, axesHandles, 36);
        for index = 1:numel(axesHandles)
            ax = axesHandles(index);
            verifyEqual(testCase, ax.XColor, [0, 0, 0]);
            verifyEqual(testCase, ax.YColor, [0, 0, 0]);
            verifyEqual(testCase, ax.ZColor, [0, 0, 0]);
            verifyEqual(testCase, ax.FontName, 'Times');
            verifyEqual(testCase, ax.FontSize, 14);
            verifyEqual(testCase, ax.FontUnits, 'points');
            verifyGreaterThanOrEqual(testCase, ax.YLim(1), 0);
        end
        labels = findall(fig, 'Type', 'text');
        for index = 1:numel(labels)
            if isempty(labels(index).String), continue; end
            verifyEqual(testCase, labels(index).Color, [0, 0, 0]);
            verifyEqual(testCase, labels(index).FontName, 'Times');
            verifyEqual(testCase, labels(index).FontSize, 14);
        end
        clear figureCleanup;
    end
end

function verifyField(testCase, field, study, planner, column)
%VERIFYFIELD Сопоставляет серии с таблицами и проверяет геометрию трёх осей.
    drawnow;
    ax = findall(field, 'Tag', 'StudyDataAxes');
    verifyNumElements(testCase, ax, 1);
    key = findall(field, 'Tag', 'StudyLegend');
    verifyEqual(testCase, key.Location, 'northeast');
    verifyGreaterThanOrEqual(testCase, key.Position(1:2), ax.Position(1:2) - 1);
    verifyLessThanOrEqual(testCase, key.Position(1:2) + key.Position(3:4), ...
        ax.Position(1:2) + ax.Position(3:4) + 1);
    verifyEqual(testCase, string(ax.XAxis.Visible), "off");
    if strcmp(planner, 'astar')
        verifyNumElements(testCase, findobj(ax, 'Type', 'line'), 1);
        verifyNumElements(testCase, findobj(ax, 'Type', 'bar'), 2);
        verifyEqual(testCase, ax.YLim(1), 0);
    else
        verifyNumElements(testCase, findobj(ax, 'Type', 'line'), 3);
        verifyEmpty(testCase, findobj(ax, 'Type', 'bar'));
    end
    verifyNumElements(testCase, findall(field, '-regexp', 'Tag', '^StudyParameterAxis_'), 3);
    parameters = fieldnames(study.methods.(planner));
    for parameter = 1:numel(parameters)
        name = parameters{parameter};
        summary = study.methods.(planner).(name).table;
        series = findobj(ax, 'Tag', ['StudySeries_', name]);
        values = summary.(column);
        values(~isfinite(values)) = NaN;
        verifyEqual(testCase, series.YData(:), values);
        verifyEqual(testCase, series.UserData.Column, column);
        verifyEqual(testCase, series.UserData.ParameterValue, summary.ParameterValue);
        scale = findall(field, 'Tag', ['StudyParameterAxis_', name]);
        verifyEqual(testCase, scale.Position([1, 3]), ax.Position([1, 3]), 'AbsTol', 1e-10);
        verifyEqual(testCase, scale.Position(2), ax.Position(2) - 86*(parameter - 1), 'AbsTol', 1e-10);
        verifyEqual(testCase, string(scale.XAxis.Visible), "on");
        verifyEqual(testCase, string(scale.YAxis.Visible), "off");
        verifyEqual(testCase, scale.XLabel.String, study.variants.(planner).(name).label);
        verifyEqual(testCase, scale.XScale, 'linear');
        if isnumeric(summary.ParameterValue)
            verifyEqual(testCase, series.LineStyle, '-');
            verifyEqual(testCase, series.Marker, 'none');
        else
            verifyEqual(testCase, series.Type, 'bar');
            mapping = scale.UserData;
            projected = mapping.Coordinates(1) + series.XEndPoints*diff(mapping.Coordinates);
            verifyEqual(testCase, projected(:), (1:height(summary))', 'AbsTol', 1e-12);
        end
        verifyEqual(testCase, string(series.Clipping), "on");
        if ~isnumeric(summary.ParameterValue)
            if isfield(study.variants.(planner).(name), 'tickLabels')
                verifyEqual(testCase, scale.XTickLabel, study.variants.(planner).(name).tickLabels(:));
            end
        end
    end
end

function testLinearPositionsIgnoreLegacyLogSetting(testCase)
%TESTLINEARPOSITIONSIGNORELEGACYLOGSETTING Использует линейные оси и для старых MAT с scale=log.
    study = testCase.TestData.study;
    summary = study.methods.voronoi.clearanceWeight.table([1, 1, 2], :);
    summary.ParameterValue = [1; 10; 100];
    study.methods.voronoi.clearanceWeight.table = summary;
    visible = get(groot, 'DefaultFigureVisible');
    set(groot, 'DefaultFigureVisible', 'off');
    visibilityCleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', visible));
    fig = viz.plotParameterStudy(study, 'voronoi');
    figureCleanup = onCleanup(@() close(fig));
    field = findobj(fig, 'Tag', 'StudyField_MeanPlanningTime_s');
    ax = findall(field, 'Tag', 'StudyDataAxes');
    series = findobj(ax, 'Tag', 'StudySeries_clearanceWeight');
    scale = findall(field, 'Tag', 'StudyParameterAxis_clearanceWeight');
    verifyEqual(testCase, diff(series.XData), [9/99, 90/99], 'AbsTol', 1e-12);
    verifyEqual(testCase, series.DataTipTemplate.DataTipRows(1).Value, [1; 10; 100]);
    verifyEqual(testCase, scale.XScale, 'linear');
    verifyEqual(testCase, scale.XLim, [1, 100], 'AbsTol', 1e-12);
    verifyEqual(testCase, scale.XTick, [1, 50.5, 100], 'AbsTol', 1e-12);
    ax.XLim = [0.25, 0.75];
    drawnow;
    verifyEqual(testCase, scale.XLim, [25.75, 75.25], 'AbsTol', 1e-12);
end

function testMissingAndInfiniteEstimates(testCase)
%TESTMISSINGANDINFINITEESTIMATES Не рисует нулевые столбцы вместо отсутствующих оценок.
    study = testCase.TestData.study;
    parameters = fieldnames(study.methods.rrt);
    for index = 1:numel(parameters)
        study.methods.rrt.(parameters{index}).table.VariancePlanningTime_s(:) = NaN;
    end
    study.methods.rrt.stepSize.table.MeanPlanningTime_s = [Inf; NaN];
    visible = get(groot, 'DefaultFigureVisible');
    set(groot, 'DefaultFigureVisible', 'off');
    visibilityCleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', visible));
    fig = viz.plotParameterStudy(study, 'rrt');
    figureCleanup = onCleanup(@() close(fig));
    field = findobj(fig, 'Tag', 'StudyField_MeanPlanningTime_s');
    ax = findall(field, 'Tag', 'StudyDataAxes');
    verifyNumElements(testCase, findobj(ax, 'Tag', 'StudyInfiniteEstimate'), 1);
    field = findobj(fig, 'Tag', 'StudyField_VariancePlanningTime_s');
    ax = findall(field, 'Tag', 'StudyDataAxes');
    verifyNumElements(testCase, findobj(ax, 'Tag', 'StudyMissingEstimate'), 3);
    verifyEmpty(testCase, findobj(ax, 'Tag', 'StudyInfiniteEstimate'));
    verifyField(testCase, field, study, 'rrt', 'VariancePlanningTime_s');
end

function testCategoryBarsAndUserNotation(testCase)
%TESTCATEGORYBARSANDUSERNOTATION Проверяет разделение столбцов и сохранение обозначений с TeX.
    study = testCase.TestData.study;
    study.variants.astar.heuristicType.label = 'H(n)';
    study.variants.astar.heuristicWeight.label = 'w';
    study.variants.astar.tieBreaking.label = 'K(n)';
    visible = get(groot, 'DefaultFigureVisible');
    set(groot, 'DefaultFigureVisible', 'off');
    visibilityCleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', visible));
    fig = viz.plotParameterStudy(study, 'astar');
    figureCleanup = onCleanup(@() close(fig));
    field = findobj(fig, 'Tag', 'StudyField_MeanPlanningTime_s');
    ax = findall(field, 'Tag', 'StudyDataAxes');
    verifyEqual(testCase, ax.YLabel.String, 'M_{x}');
    field = findobj(fig, 'Tag', 'StudyField_VariancePlanningTime_s');
    verifyEqual(testCase, findall(field, 'Tag', 'StudyDataAxes').YLabel.String, 'D_{x}');
    first = findobj(ax, 'Tag', 'StudySeries_heuristicType');
    second = findobj(ax, 'Tag', 'StudySeries_tieBreaking');
    firstWidth = first.BarWidth*min(diff(first.XData));
    secondWidth = second.BarWidth*min(diff(second.XData));
    verifyGreaterThan(testCase, second.XEndPoints - secondWidth/2, first.XEndPoints + firstWidth/2);
    verifyEqual(testCase, ax.TickLabelInterpreter, 'tex');
    scale = findall(ax.Parent, 'Tag', 'StudyParameterAxis_heuristicType');
    verifyEqual(testCase, scale.XLabel.String, 'H(n)');
    verifyEqual(testCase, scale.XLabel.Interpreter, 'tex');
end

function testSavedFigureTabs(testCase)
%TESTSAVEDFIGURETABS Проверяет вкладки, масштабирование и выравнивание после открытия FIG.
    study = testCase.TestData.study;
    filename = [tempname, '.fig'];
    fileCleanup = onCleanup(@() delete(filename));
    visible = get(groot, 'DefaultFigureVisible');
    set(groot, 'DefaultFigureVisible', 'off');
    visibilityCleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', visible));
    fig = viz.plotParameterStudy(study, 'astar');
    figureCleanup = onCleanup(@() close(fig));
    savefig(fig, filename);
    clear figureCleanup;
    fig = openfig(filename, 'invisible');
    figureCleanup = onCleanup(@() close(fig));
    verifyNumElements(testCase, findall(fig, 'Type', 'uitab'), 5);
    group = findobj(fig, 'Tag', 'StudyTabs');
    group.SelectedTab = findobj(group, 'Tag', 'PathLength_m');
    feval(group.SelectionChangedFcn, group, []);
    field = findobj(group.SelectedTab, 'Tag', 'StudyField_MeanPathLength_m');
    verifyField(testCase, field, study, 'astar', 'MeanPathLength_m');
    ax = findall(field, 'Tag', 'StudyDataAxes');
    scale = findall(field, 'Tag', 'StudyParameterAxis_heuristicWeight');
    ax.XLim = [0.25, 0.75];
    drawnow;
    verifyEqual(testCase, scale.XLim, [0.625, 0.875], 'AbsTol', 1e-12);
    ax.XLim = [0, 1];
    fig.Position = [50, 50, 1280, 850];
    feval(fig.SizeChangedFcn, fig, []);
    verifyField(testCase, field, study, 'astar', 'MeanPathLength_m');
    verifyEqual(testCase, field.Position(3), 640, 'AbsTol', 1e-10);
    verifyLessThanOrEqual(testCase, ax.Position(1) + ax.Position(3), field.Position(3));
    verifyLessThanOrEqual(testCase, ax.Position(2) + ax.Position(4), field.Position(4));
    group.SelectedTab = findobj(group, 'Tag', 'Probability');
    feval(group.SelectionChangedFcn, group, []);
    field = findobj(group.SelectedTab, 'Tag', 'StudyField_Probability');
    verifyField(testCase, field, study, 'astar', 'Probability');
end
