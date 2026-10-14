function tests = testDemoPlots
%TESTDEMOPLOTS Проверяет компоновку окон demo и данные общих графиков.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Сохраняет окружение и подключает исходники.
    testCase.TestData.root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.rng = rng;
    addpath(genpath(fullfile(testCase.TestData.root, 'src')));
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает окружение MATLAB.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.rng);
end

function teardown(~)
%TEARDOWN Закрывает созданные тестами окна.
    close all;
end

function testDemoHasThreeWindowsWithSixPlots(testCase)
%TESTDEMOHASTHREEWINDOWSWITHSIXPLOTS Проверяет реальный запуск scripts/run_demo.m.
    results = runDemo(testCase.TestData.root);
    figures = findall(groot, 'Type', 'figure');
    verifyNumElements(testCase, figures, 3);
    for index = 1:3
        result = results{index};
        fig = findobj(groot, 'Type', 'figure', 'Name', result.displayName);
        assertNumElements(testCase, fig, 1);
        verifyNumElements(testCase, findobj(fig, 'Type', 'axes'), 6);
        mapAxes = findobj(fig, 'Tag', 'DemoMapAxes');
        verifyEqual(testCase, mapAxes.Layout.Tile, 1);
        verifyEqual(testCase, mapAxes.Layout.TileSpan, [3, 1]);
        errorAxes = findobj(fig, 'Tag', 'DemoErrorAxes');
        verifyEqual(testCase, errorAxes.Layout.Tile, 7);
        verifyNumElements(testCase, findobj(errorAxes, 'Type', 'line'), 2);
        componentNames = {'x', 'y'};
        for component = 1:2
            curve = findobj(errorAxes, 'DisplayName', componentNames{component});
            verifyEqual(testCase, curve.YData, ...
                result.simLog.x(component, :) - result.simLog.xd(component, :));
        end
        derivatives = {result.simLog.firstDerivative, result.simLog.secondDerivative, ...
            result.simLog.thirdDerivative, result.simLog.fourthDerivative};
        for order = 1:4
            ax = findobj(fig, 'Tag', sprintf('DemoDerivativeAxes%d', order));
            verifyEqual(testCase, ax.Layout.Tile, 2*order);
            verifyEqual(testCase, ax.XLim, [result.simLog.t(1), result.simLog.t(end)]);
            verifyNumElements(testCase, findobj(ax, 'Type', 'line'), 2);
            for component = 1:2
                curve = findobj(ax, 'DisplayName', componentNames{component});
                verifyEqual(testCase, curve.YData, derivatives{order}(component, :));
            end
        end
    end
end

function results = runDemo(root)
%RUNDEMO Изолирует команду clear скрипта от рабочего пространства теста.
    run(fullfile(root, 'scripts', 'run_demo.m'));
    results = {rrtResult, astarResult, voronoiResult};
end

function testFailedPlannerKeepsEmptyDiagnostics(testCase)
%TESTFAILEDPLANNERKEEPSEMPTYDIAGNOSTICS Сохраняет шесть графиков при отказе поиска.
    result = struct('displayName', 'Нет маршрута', 'tree', [], 'path', [], ...
        'simLog', [], 'rawPath', [], 'rrtPath', []);
    fig = viz.plotPlannerResult(config.defaultScenario(), result);
    verifyNumElements(testCase, findall(groot, 'Type', 'figure'), 1);
    verifyNumElements(testCase, findobj(fig, 'Type', 'axes'), 6);
    ax = findobj(fig, 'Tag', 'DemoErrorAxes');
    verifyEmpty(testCase, findobj(ax, 'Type', 'line'));
    for order = 1:4
        ax = findobj(fig, 'Tag', sprintf('DemoDerivativeAxes%d', order));
        verifyEmpty(testCase, findobj(ax, 'Type', 'line'));
    end
end

function testTrackingErrorIgnoresAltitude(testCase)
%TESTTRACKINGERRORIGNORESALTITUDE Выводит только знаковые ошибки x и y.
    fig = figure('Visible', 'off');
    ax = axes(fig);
    log = struct('t', [0, 1, 2], 'x', [1, 2, 3; -3, -2, -1; 1000, 1000, 1000], ...
        'xd', [0, 1, 2; -2, -1, 0; 0, 0, 0]);
    viz.plotTrackingError(ax, log);
    verifyNumElements(testCase, findobj(ax, 'Type', 'line'), 2);
    curve = findobj(ax, 'DisplayName', 'x');
    verifyEqual(testCase, curve.YData, [1, 1, 1]);
    curve = findobj(ax, 'DisplayName', 'y');
    verifyEqual(testCase, curve.YData, [-1, -1, -1]);
    viz.plotTrackingError(ax, []);
    verifyEmpty(testCase, findobj(ax, 'Type', 'line'));
end
