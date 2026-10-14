function tests = testPlannerApp
%TESTPLANNERAPP Проверяет выбор метода и графики через обработчики интерфейса.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%SETUPONCE Подключает приложение и сохраняет окружение MATLAB.
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    testCase.TestData.rng = rng;
    addpath(genpath(fullfile(root, 'src')));
    addpath(fullfile(root, 'apps'));
    assumeTrue(testCase, exist('minsnappolytraj', 'file') == 2);
end

function teardownOnce(testCase)
%TEARDOWNONCE Восстанавливает окружение MATLAB.
    path(testCase.TestData.oldPath);
    rng(testCase.TestData.rng);
end

function setup(testCase)
%SETUP Создаёт отдельное окно для каждой проверки.
    app = QuadrotorRRTApp();
    app.UIFigure.Visible = 'off';
    testCase.TestData.app = app;
end

function teardown(testCase)
%TEARDOWN Закрывает окно приложения после проверки.
    close(testCase.TestData.app.UIFigure);
end

function testPlannerSpecificControls(testCase)
%TESTPLANNERSPECIFICCONTROLS Проверяет доступность настроек выбранного метода.
    fig = testCase.TestData.app.UIFigure;
    selector = findobj(fig, 'Tag', 'PlannerDropDown');
    step = findobj(fig, 'Tag', 'StepSizeField');
    smoothing = findobj(fig, 'Tag', 'SmoothIterField');
    weight = findobj(fig, 'Tag', 'ClearanceWeightField');
    heuristicWeight = findobj(fig, 'Tag', 'HeuristicWeightField');
    verifyEqual(testCase, selector.ItemsData, {'rrt', 'astar', 'voronoi'});
    verifyEqual(testCase, char(step.Enable), 'on');
    verifyEqual(testCase, char(smoothing.Enable), 'on');
    verifyEqual(testCase, char(weight.Enable), 'off');
    verifyEqual(testCase, char(heuristicWeight.Enable), 'off');
    selector.Value = 'astar';
    selector.ValueChangedFcn(selector, []);
    verifyEqual(testCase, char(step.Enable), 'off');
    verifyEqual(testCase, char(smoothing.Enable), 'off');
    verifyEqual(testCase, char(weight.Enable), 'off');
    verifyEqual(testCase, char(heuristicWeight.Enable), 'on');
    selector.Value = 'voronoi';
    selector.ValueChangedFcn(selector, []);
    verifyEqual(testCase, char(step.Enable), 'off');
    verifyEqual(testCase, char(smoothing.Enable), 'off');
    verifyEqual(testCase, char(weight.Enable), 'on');
    verifyEqual(testCase, char(heuristicWeight.Enable), 'off');
end

function testAllPlannerGraphsAndValues(testCase)
%TESTALLPLANNERGRAPHSANDVALUES Сравнивает графики с журналом каждого метода.
    fig = testCase.TestData.app.UIFigure;
    selector = findobj(fig, 'Tag', 'PlannerDropDown');
    button = findobj(fig, 'Tag', 'PlanningButton');
    table = findobj(fig, 'Tag', 'ResultTable');
    tabs = findobj(fig, 'Tag', 'ResultTabs');
    scenario = config.defaultScenario();
    originalFontSize = selector.FontSize;
    componentSelector = findobj(fig, 'Tag', 'DerivativeComponentDropDown');
    verifyEqual(testCase, componentSelector.ItemsData, {'all', 'x', 'y'});
    verifyNumElements(testCase, tabs.Children, 3);
    verifyEmpty(testCase, findobj(fig, 'Tag', 'ErrorNormAxes'));
    verifyEmpty(testCase, findobj(fig, 'Tag', 'MotorAxes'));
    verifyEmpty(testCase, findobj(tabs, 'Title', 'Тяги двигателей'));
    for planner = {'rrt', 'astar', 'voronoi'}
        caseScenario = scenario;
        if strcmp(planner{1}, 'astar')
            caseScenario.astar.heuristicType = 'octile';
            caseScenario.astar.heuristicWeight = 0.75;
            caseScenario.astar.tieBreaking = 'smallerg';
            field = findobj(fig, 'Tag', 'HeuristicTypeDropDown'); field.Value = 'octile';
            field = findobj(fig, 'Tag', 'HeuristicWeightField'); field.Value = 0.75;
            field = findobj(fig, 'Tag', 'TieBreakingDropDown'); field.Value = 'smallerg';
        elseif strcmp(planner{1}, 'voronoi')
            caseScenario.voronoi.clearanceWeight = 30;
            caseScenario.voronoi.clearanceExponent = 1.5;
            caseScenario.voronoi.connectionCount = 5;
            field = findobj(fig, 'Tag', 'ClearanceWeightField'); field.Value = 30;
            field = findobj(fig, 'Tag', 'ClearanceExponentField'); field.Value = 1.5;
            field = findobj(fig, 'Tag', 'ConnectionCountField'); field.Value = 5;
        end
        rng(121314);
        expected = runPlannerExperiment(caseScenario, planner{1});
        selector.Value = planner{1};
        selector.ValueChangedFcn(selector, []);
        rng(121314);
        button.ButtonPushedFcn(button, []);
        assertEqual(testCase, table.Data{1, 1}, expected.displayName);
        assertTrue(testCase, table.Data{1, 2});
        verifyEqual(testCase, char(button.Enable), 'on');
        verifyEqual(testCase, selector.FontSize, originalFontSize);
        log = expected.simLog;
        componentAxes = findobj(fig, 'Tag', 'ErrorComponentAxes');
        componentLines = flipud(findobj(componentAxes, 'Type', 'line'));
        verifyNumElements(testCase, componentLines, 2);
        verifyEqual(testCase, componentAxes.XLim, [log.t(1), log.t(end)]);
        for component = 1:2
            verifyEqual(testCase, componentLines(component).XData, log.t);
            verifyEqual(testCase, componentLines(component).YData, log.x(component, :)-log.xd(component, :));
        end
        verifyEmpty(testCase, findobj(componentAxes, 'DisplayName', 'z'));
        actual = {log.firstDerivative, log.secondDerivative, log.thirdDerivative, log.fourthDerivative};
        reference = {log.vd, log.ad, log.jerk, log.snap};
        for order = 1:4
            ax = findobj(fig, 'Tag', sprintf('DerivativeAxes%d', order));
            verifyNumElements(testCase, findobj(ax, 'Type', 'line'), 4);
            verifyEqual(testCase, ax.XLim, [log.t(1), log.t(end)]);
            componentNames = {'x', 'y'};
            verifyEmpty(testCase, findobj(ax, 'DisplayName', 'z (ЛА)'));
            verifyEmpty(testCase, findobj(ax, 'DisplayName', 'z (опорная)'));
            for component = 1:2
                realLine = findobj(ax, 'DisplayName', sprintf('%s (ЛА)', componentNames{component}));
                refLine = findobj(ax, 'DisplayName', sprintf('%s (опорная)', componentNames{component}));
                verifyEqual(testCase, realLine.YData, actual{order}(component, :));
                verifyEqual(testCase, refLine.YData, reference{order}(component, :));
                verifyEqual(testCase, refLine.LineStyle, '--');
            end
        end
    end
    componentSelector = findobj(fig, 'Tag', 'DerivativeComponentDropDown');
    componentSelector.Value = 'y';
    componentSelector.ValueChangedFcn(componentSelector, []);
    for order = 1:4
        ax = findobj(fig, 'Tag', sprintf('DerivativeAxes%d', order));
        verifyNumElements(testCase, findobj(ax, 'Type', 'line'), 2);
        verifyEmpty(testCase, findobj(ax, 'DisplayName', 'x (ЛА)'));
        verifyNotEmpty(testCase, findobj(ax, 'DisplayName', 'y (ЛА)'));
    end
end

function testFailedSearchAndResetClearGraphs(testCase)
%TESTFAILEDSEARCHANDRESETCLEARGRAPHS Не оставляет графики предыдущего результата.
    fig = testCase.TestData.app.UIFigure;
    selector = findobj(fig, 'Tag', 'PlannerDropDown');
    button = findobj(fig, 'Tag', 'PlanningButton');
    selector.Value = 'astar';
    selector.ValueChangedFcn(selector, []);
    button.ButtonPushedFcn(button, []);
    obstacles = findobj(fig, 'Tag', 'ObstaclesTable');
    obstacles.Data = [500, 0, 10, 1000];
    button.ButtonPushedFcn(button, []);
    table = findobj(fig, 'Tag', 'ResultTable');
    verifyFalse(testCase, table.Data{1, 2});
    tags = {'ErrorComponentAxes', ...
        'DerivativeAxes1', 'DerivativeAxes2', 'DerivativeAxes3', 'DerivativeAxes4'};
    for index = 1:numel(tags)
        ax = findobj(fig, 'Tag', tags{index});
        verifyEmpty(testCase, findobj(ax, 'Type', 'line'));
    end
    reset = findobj(fig, 'Text', 'По умолчанию');
    reset.ButtonPushedFcn(reset, []);
    verifyEmpty(testCase, table.Data);
    verifyEqual(testCase, selector.Value, 'rrt');
    verifyEqual(testCase, char(button.Enable), 'on');
end
