classdef QuadrotorRRTApp < matlab.apps.AppBase
    %QUADROTORRRTAPP Планирование и моделирование траекторий квадрокоптера.

    properties (Access = public)
        UIFigure matlab.ui.Figure
    end

    properties (Access = private)
        RootDir char
        CurrentResult

        MainGrid matlab.ui.container.GridLayout
        LeftPanel matlab.ui.container.Panel
        RightPanel matlab.ui.container.Panel

        PlotAxes matlab.ui.control.UIAxes
        ErrorComponentAxes matlab.ui.control.UIAxes
        DerivativeAxes
        PlannerDropDown matlab.ui.control.DropDown
        DerivativeComponentDropDown matlab.ui.control.DropDown
        PlanningButton matlab.ui.control.Button
        ClearanceWeightField matlab.ui.control.NumericEditField
        ClearanceExponentField matlab.ui.control.NumericEditField
        ConnectionCountField matlab.ui.control.NumericEditField
        HeuristicWeightField matlab.ui.control.NumericEditField
        HeuristicTypeDropDown matlab.ui.control.DropDown
        TieBreakingDropDown matlab.ui.control.DropDown
        PlannerPanel matlab.ui.container.Panel
        AStarPanel matlab.ui.container.Panel
        VoronoiPanel matlab.ui.container.Panel
        ResultTable matlab.ui.control.Table
        StatusLabel matlab.ui.control.Label

        WidthField matlab.ui.control.NumericEditField
        HeightField matlab.ui.control.NumericEditField
        ResolutionField matlab.ui.control.NumericEditField
        StartXField matlab.ui.control.NumericEditField
        StartYField matlab.ui.control.NumericEditField
        GoalXField matlab.ui.control.NumericEditField
        GoalYField matlab.ui.control.NumericEditField
        SafetyRadiusField matlab.ui.control.NumericEditField

        StepSizeField matlab.ui.control.NumericEditField
        GoalBiasField matlab.ui.control.NumericEditField
        MaxIterField matlab.ui.control.NumericEditField
        GoalThresholdField matlab.ui.control.NumericEditField
        CollisionStepField matlab.ui.control.NumericEditField
        SmoothIterField matlab.ui.control.NumericEditField
        DenseStepField matlab.ui.control.NumericEditField

        VRefField matlab.ui.control.NumericEditField
        DtField matlab.ui.control.NumericEditField
        ZRefField matlab.ui.control.NumericEditField
        MaxErrorField matlab.ui.control.NumericEditField

        ObstaclesTable matlab.ui.control.Table
    end

    methods (Access = public)

        function app = QuadrotorRRTApp()
            app.RootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(genpath(fullfile(app.RootDir, 'src')));

            app.createComponents();
            app.loadDefaultScenario();
        end

    end

    methods (Access = private)

        function createComponents(app)
            screen = get(groot, 'ScreenSize');
            app.UIFigure = uifigure( ...
                'Name', 'Планирование траектории квадрокоптера', ...
                'Position', [screen(1) + 40, screen(2) + 40, ...
                min(1440, screen(3) - 80), min(900, screen(4) - 100)]);

            app.MainGrid = uigridlayout(app.UIFigure, [1 2]);
            app.MainGrid.ColumnWidth = {570, '1x'};
            app.MainGrid.RowHeight = {'1x'};
            app.MainGrid.Padding = [8 8 8 8];
            app.MainGrid.ColumnSpacing = 8;

            app.LeftPanel = uipanel(app.MainGrid, ...
                'Title', 'Параметры');

            app.RightPanel = uipanel(app.MainGrid, ...
                'Title', 'Результаты');

            %% ===================== ЛЕВАЯ ПАНЕЛЬ =====================

            leftGrid = uigridlayout(app.LeftPanel, [2 1]);
            leftGrid.RowHeight = {'1x', 120};
            leftGrid.ColumnWidth = {'1x'};
            leftGrid.Padding = [8 8 8 8];
            leftGrid.RowSpacing = 8;

            paramPanel = uipanel(leftGrid, ...
                'Title', 'Параметры расчёта');
            paramPanel.Layout.Row = 1;

            actionPanel = uipanel(leftGrid, ...
                'Title', 'Управление');
            actionPanel.Layout.Row = 2;

            if isprop(paramPanel, 'Scrollable')
                paramPanel.Scrollable = 'on';
            end

            paramGrid = uigridlayout(paramPanel, [5 1]);
            paramGrid.RowHeight = {140, 145, 175, 95, 160};
            paramGrid.ColumnWidth = {'1x'};
            paramGrid.Padding = [8 8 8 8];
            paramGrid.RowSpacing = 8;
            paramGrid.Scrollable = 'on';

            plannerPanel = uipanel(paramGrid, 'Title', 'Метод планирования');
            app.PlannerPanel = plannerPanel;
            plannerPanel.Layout.Row = 1;
            plannerGrid = uigridlayout(plannerPanel, [2 1]);
            plannerGrid.RowHeight = {26, '1x'};
            plannerGrid.ColumnWidth = {'1x'};
            plannerGrid.Padding = [8 6 8 6];
            plannerGrid.RowSpacing = 5;

            selector = uigridlayout(plannerGrid, [1 2]);
            selector.ColumnWidth = {105, '1x'};
            selector.Padding = [0 0 0 0];
            addLabel(selector, 'Метод');
            app.PlannerDropDown = uidropdown(selector, ...
                'Items', {'RRT', 'A*', 'Вороной (GVD)'}, ...
                'ItemsData', {'rrt', 'astar', 'voronoi'}, 'Value', 'rrt', ...
                'Tag', 'PlannerDropDown', ...
                'ValueChangedFcn', @(~,~) app.updatePlannerControls());
            app.PlannerDropDown.Layout.Row = 1;
            app.PlannerDropDown.Layout.Column = 2;
            app.AStarPanel = uipanel(plannerGrid, 'BorderType', 'none');
            app.AStarPanel.Layout.Row = 2;
            app.VoronoiPanel = uipanel(plannerGrid, 'BorderType', 'none');
            app.VoronoiPanel.Layout.Row = 2;
            astarGrid = uigridlayout(app.AStarPanel, [2 4]);
            astarGrid.ColumnWidth = {105, '1x', 105, '1x'};
            astarGrid.RowHeight = {26, 26};
            astarGrid.Padding = [0 0 0 0];
            astarGrid.RowSpacing = 5;
            addLabel(astarGrid, 'Эвристика');
            app.HeuristicTypeDropDown = uidropdown(astarGrid, ...
                'Items', {'Евклидова', 'Октильная'}, 'ItemsData', {'euclidean', 'octile'}, ...
                'Tag', 'HeuristicTypeDropDown');
            addLabel(astarGrid, 'Вес w');
            app.HeuristicWeightField = addNum(astarGrid, 1);
            app.HeuristicWeightField.Limits = [eps 1];
            app.HeuristicWeightField.Tag = 'HeuristicWeightField';
            addLabel(astarGrid, 'Равенства');
            app.TieBreakingDropDown = uidropdown(astarGrid, ...
                'Items', {'Больший g', 'Меньший g'}, 'ItemsData', {'largerg', 'smallerg'}, ...
                'Tag', 'TieBreakingDropDown');
            app.TieBreakingDropDown.Layout.Row = 2;
            app.TieBreakingDropDown.Layout.Column = [2 4];
            voronoiGrid = uigridlayout(app.VoronoiPanel, [2 4]);
            voronoiGrid.ColumnWidth = {105, '1x', 105, '1x'};
            voronoiGrid.RowHeight = {26, 26};
            voronoiGrid.Padding = [0 0 0 0];
            voronoiGrid.RowSpacing = 5;
            addLabel(voronoiGrid, 'Вес отступа');
            app.ClearanceWeightField = addNum(voronoiGrid, 9);
            app.ClearanceWeightField.Tag = 'ClearanceWeightField';
            app.ClearanceWeightField.Limits = [eps Inf];
            addLabel(voronoiGrid, 'Степень p');
            app.ClearanceExponentField = addNum(voronoiGrid, 1);
            app.ClearanceExponentField.Limits = [eps Inf];
            app.ClearanceExponentField.Tag = 'ClearanceExponentField';
            addLabel(voronoiGrid, 'Подключения K');
            app.ConnectionCountField = addNum(voronoiGrid, 3);
            app.ConnectionCountField.Limits = [1 Inf];
            app.ConnectionCountField.Tag = 'ConnectionCountField';

            %% ===================== РАБОЧАЯ ОБЛАСТЬ =====================

            mapPanel = uipanel(paramGrid, ...
                'Title', 'Рабочая область');
            mapPanel.Layout.Row = 2;

            mapGrid = uigridlayout(mapPanel, [4 4]);
            mapGrid.RowHeight = {24, 24, 24, 24};
            mapGrid.ColumnWidth = {105, '1x', 105, '1x'};
            mapGrid.Padding = [8 6 8 6];
            mapGrid.RowSpacing = 5;
            mapGrid.ColumnSpacing = 10;

            addLabel(mapGrid, 'Ширина, м');
            app.WidthField = addNum(mapGrid, 1000);

            addLabel(mapGrid, 'Высота, м');
            app.HeightField = addNum(mapGrid, 1000);

            addLabel(mapGrid, 'Разрешение, м');
            app.ResolutionField = addNum(mapGrid, 1);

            addLabel(mapGrid, 'Отступ, м');
            app.SafetyRadiusField = addNum(mapGrid, 5);

            addLabel(mapGrid, 'Старт X');
            app.StartXField = addNum(mapGrid, 50);

            addLabel(mapGrid, 'Старт Y');
            app.StartYField = addNum(mapGrid, 50);

            addLabel(mapGrid, 'Цель X');
            app.GoalXField = addNum(mapGrid, 950);

            addLabel(mapGrid, 'Цель Y');
            app.GoalYField = addNum(mapGrid, 950);

            %% ===================== ПАРАМЕТРЫ RRT =====================

            rrtPanel = uipanel(paramGrid, ...
                'Title', 'Поиск и обработка маршрута');
            rrtPanel.Layout.Row = 3;

            rrtGrid = uigridlayout(rrtPanel, [4 4]);
            rrtGrid.RowHeight = {26, 26, 26, 26};
            rrtGrid.ColumnWidth = {105, '1x', 105, '1x'};
            rrtGrid.Padding = [8 6 8 6];
            rrtGrid.RowSpacing = 5;
            rrtGrid.ColumnSpacing = 10;

            addLabel(rrtGrid, 'Шаг, м');
            app.StepSizeField = addNum(rrtGrid, 50);
            app.StepSizeField.Tag = 'StepSizeField';

            addLabel(rrtGrid, 'Goal bias');
            app.GoalBiasField = addNum(rrtGrid, 0.1);

            addLabel(rrtGrid, 'Итерации');
            app.MaxIterField = addNum(rrtGrid, 6000);

            addLabel(rrtGrid, 'Порог цели');
            app.GoalThresholdField = addNum(rrtGrid, 50);

            addLabel(rrtGrid, 'Шаг проверки');
            app.CollisionStepField = addNum(rrtGrid, 1);

            addLabel(rrtGrid, 'Сокращения');
            app.SmoothIterField = addNum(rrtGrid, 300);
            app.SmoothIterField.Tag = 'SmoothIterField';

            addLabel(rrtGrid, 'Шаг пути');
            app.DenseStepField = addNum(rrtGrid, 1);

            uilabel(rrtGrid, 'Text', '');
            uilabel(rrtGrid, 'Text', '');

            %% ===================== ПАРАМЕТРЫ СИМУЛЯЦИИ =====================

            simPanel = uipanel(paramGrid, ...
                'Title', 'Параметры симуляции');
            simPanel.Layout.Row = 4;

            simGrid = uigridlayout(simPanel, [2 4]);
            simGrid.RowHeight = {25, 25};
            simGrid.ColumnWidth = {105, '1x', 105, '1x'};
            simGrid.Padding = [8 6 8 6];
            simGrid.RowSpacing = 5;
            simGrid.ColumnSpacing = 10;

            addLabel(simGrid, 'v ref, м/с');
            app.VRefField = addNum(simGrid, 10);

            addLabel(simGrid, 'dt, с');
            app.DtField = addNum(simGrid, 0.02);

            addLabel(simGrid, 'z ref, м');
            app.ZRefField = addNum(simGrid, 0);

            addLabel(simGrid, 'Макс. ошибка, м');
            app.MaxErrorField = addNum(simGrid, 25);
            app.MaxErrorField.Tooltip = 'Допустимая конечная ошибка слежения, м';

            %% ===================== ПРЕПЯТСТВИЯ =====================

            obstaclePanel = uipanel(paramGrid, ...
                'Title', 'Препятствия [x, y, ширина, высота]');
            obstaclePanel.Layout.Row = 5;

            obstacleGrid = uigridlayout(obstaclePanel, [2 1]);
            obstacleGrid.RowHeight = {'1x', 30};
            obstacleGrid.ColumnWidth = {'1x'};
            obstacleGrid.Padding = [8 6 8 6];
            obstacleGrid.RowSpacing = 5;

            app.ObstaclesTable = uitable(obstacleGrid);
            app.ObstaclesTable.Tag = 'ObstaclesTable';
            app.ObstaclesTable.ColumnName = {'x', 'y', 'w', 'h'};
            app.ObstaclesTable.ColumnEditable = [true true true true];
            app.ObstaclesTable.Layout.Row = 1;
            app.ObstaclesTable.Layout.Column = 1;

            obstacleButtonGrid = uigridlayout(obstacleGrid, [1 4]);
            obstacleButtonGrid.Layout.Row = 2;
            obstacleButtonGrid.Layout.Column = 1;
            obstacleButtonGrid.RowHeight = {26};
            obstacleButtonGrid.ColumnWidth = {'1x', '1x', '1.25x', '1x'};
            obstacleButtonGrid.Padding = [0 0 0 0];
            obstacleButtonGrid.ColumnSpacing = 6;

            uibutton(obstacleButtonGrid, ...
                'Text', 'Добавить', ...
                'ButtonPushedFcn', @(~,~) app.addObstacle());

            uibutton(obstacleButtonGrid, ...
                'Text', 'Удалить', ...
                'ButtonPushedFcn', @(~,~) app.deleteObstacle());

            uibutton(obstacleButtonGrid, ...
                'Text', 'По умолчанию', ...
                'ButtonPushedFcn', @(~,~) app.loadDefaultScenario());

            uibutton(obstacleButtonGrid, ...
                'Text', 'Очистить', ...
                'ButtonPushedFcn', @(~,~) app.clearObstacles());

            %% ===================== УПРАВЛЕНИЕ =====================

            actionGrid = uigridlayout(actionPanel, [2 3]);
            actionGrid.RowHeight = {30, '1x'};
            actionGrid.ColumnWidth = {'1.25x', '1x', '1.25x'};
            actionGrid.Padding = [8 6 8 6];
            actionGrid.RowSpacing = 4;
            actionGrid.ColumnSpacing = 8;

            app.PlanningButton = uibutton(actionGrid, ...
                'Text', 'Построить маршрут', ...
                'Tag', 'PlanningButton', ...
                'ButtonPushedFcn', @(~,~) app.runPlanning());

            uibutton(actionGrid, ...
                'Text', 'Сохранить CSV', ...
                'ButtonPushedFcn', @(~,~) app.exportCsv());

            uibutton(actionGrid, ...
                'Text', 'Сохранить сценарий', ...
                'ButtonPushedFcn', @(~,~) app.saveScenario());

            app.StatusLabel = uilabel(actionGrid, ...
                'Text', 'Готово', 'Tag', 'StatusLabel', 'WordWrap', 'on');

            app.StatusLabel.Layout.Row = 2;
            app.StatusLabel.Layout.Column = [1 3];

            %% ===================== ПРАВАЯ ПАНЕЛЬ =====================

            rightGrid = uigridlayout(app.RightPanel, [2 1]);
            rightGrid.RowHeight = {'1x', 130};
            rightGrid.ColumnWidth = {'1x'};
            rightGrid.Padding = [10 10 10 10];
            rightGrid.RowSpacing = 8;

            tabs = uitabgroup(rightGrid, 'Tag', 'ResultTabs');
            routeTab = uitab(tabs, 'Title', 'Маршрут');
            routeGrid = uigridlayout(routeTab, [1 1]);
            app.PlotAxes = uiaxes(routeGrid, 'Tag', 'MapAxes');
            title(app.PlotAxes, 'Карта и маршрут');
            xlabel(app.PlotAxes, 'X, м');
            ylabel(app.PlotAxes, 'Y, м');

            derivativeTab = uitab(tabs, 'Title', 'Производные');
            derivativeGrid = uigridlayout(derivativeTab, [2 1]);
            derivativeGrid.RowHeight = {28, '1x'};
            selectorGrid = uigridlayout(derivativeGrid, [1 3]);
            selectorGrid.ColumnWidth = {90, 140, '1x'};
            selectorGrid.Padding = [0 0 0 0];
            addLabel(selectorGrid, 'Компонента');
            app.DerivativeComponentDropDown = uidropdown(selectorGrid, ...
                'Items', {'Все', 'x', 'y'}, ...
                'ItemsData', {'all', 'x', 'y'}, 'Value', 'all', ...
                'Tag', 'DerivativeComponentDropDown', ...
                'ValueChangedFcn', @(~,~) app.plotDerivativeGraphs());
            axesGrid = uigridlayout(derivativeGrid, [2 2]);
            axesGrid.RowHeight = {'1x', '1x'};
            axesGrid.ColumnWidth = {'1x', '1x'};
            app.DerivativeAxes = gobjects(4, 1);
            derivativeNames = {'Скорость', 'Ускорение', 'Рывок', 'Snap'};
            derivativeUnits = {'м/с', 'м/с^2', 'м/с^3', 'м/с^4'};
            for order = 1:4
                ax = uiaxes(axesGrid, 'Tag', sprintf('DerivativeAxes%d', order));
                ax.Layout.Row = ceil(order / 2);
                ax.Layout.Column = mod(order - 1, 2) + 1;
                title(ax, sprintf('%d-я производная: %s', order, derivativeNames{order}));
                xlabel(ax, 'Время, с');
                ylabel(ax, derivativeUnits{order});
                app.DerivativeAxes(order) = ax;
            end
            linkaxes(app.DerivativeAxes, 'x');

            errorTab = uitab(tabs, 'Title', 'Ошибка слежения');
            errorGrid = uigridlayout(errorTab, [1 1]);
            app.ErrorComponentAxes = uiaxes(errorGrid, 'Tag', 'ErrorComponentAxes');
            viz.plotTrackingError(app.ErrorComponentAxes, []);

            app.ResultTable = uitable(rightGrid, 'Tag', 'ResultTable');
            app.ResultTable.ColumnName = { ...
                'Метод', 'Маршрут найден', 'Время расчёта, с', 'Длина пути, м', ...
                'Макс. кривизна, 1/м', 'Средняя кривизна, 1/м', ...
                'Динамически реализуем', 'Конечная ошибка, м', ...
                'Макс. ошибка, м', 'Макс. тяга, Н', 'Средняя тяга, Н'};
            app.ResultTable.ColumnWidth = 'auto';
        end

        function loadDefaultScenario(app)
            s = config.defaultScenario();
            app.PlannerDropDown.Value = 'rrt';
            app.HeuristicTypeDropDown.Value = s.astar.heuristicType;
            app.HeuristicWeightField.Value = s.astar.heuristicWeight;
            app.TieBreakingDropDown.Value = s.astar.tieBreaking;
            app.ClearanceWeightField.Value = s.voronoi.clearanceWeight;
            app.ClearanceExponentField.Value = s.voronoi.clearanceExponent;
            app.ConnectionCountField.Value = s.voronoi.connectionCount;
            app.updatePlannerControls();

            app.WidthField.Value = s.map.width;
            app.HeightField.Value = s.map.height;
            app.ResolutionField.Value = s.map.resolution;
            app.StartXField.Value = s.start2d(1);
            app.StartYField.Value = s.start2d(2);
            app.GoalXField.Value = s.goal2d(1);
            app.GoalYField.Value = s.goal2d(2);
            app.SafetyRadiusField.Value = s.safetyRadius;

            app.StepSizeField.Value = s.rrt.stepSize;
            app.GoalBiasField.Value = s.rrt.goalBias;
            app.MaxIterField.Value = s.rrt.maxIter;
            app.GoalThresholdField.Value = s.rrt.goalThreshold;
            app.CollisionStepField.Value = s.rrt.collisionCheckStep;
            app.SmoothIterField.Value = s.rrt.smoothingIterations;
            app.DenseStepField.Value = s.rrt.densePathStep;

            app.VRefField.Value = s.sim.vRef;
            app.DtField.Value = s.sim.dt;
            app.ZRefField.Value = s.sim.zRef;
            app.MaxErrorField.Value = s.sim.maxTrackingError;

            app.ObstaclesTable.Data = s.obstacles;
            app.CurrentResult = [];
            app.ResultTable.Data = {};
            app.plotSimulationGraphs();
            app.StatusLabel.Text = 'Загружены параметры по умолчанию';

            viz.plotScenario( ...
                app.PlotAxes, ...
                s.map, ...
                s.obstacles, ...
                s.start2d, ...
                s.goal2d, ...
                [], ...
                [], ...
                [], ...
                [], ...
                []);
            legend(app.PlotAxes, 'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 11);
        end

        function updatePlannerControls(app)
        %UPDATEPLANNERCONTROLS Разрешает настройки только выбранного метода.
            rrtEnabled = matlab.lang.OnOffSwitchState(strcmp(app.PlannerDropDown.Value, 'rrt'));
            fields = [app.StepSizeField, app.GoalBiasField, app.MaxIterField, ...
                app.GoalThresholdField, app.SmoothIterField];
            set(fields, 'Enable', rrtEnabled);
            voronoiEnabled = matlab.lang.OnOffSwitchState(strcmp(app.PlannerDropDown.Value, 'voronoi'));
            set([app.ClearanceWeightField, app.ClearanceExponentField, app.ConnectionCountField], ...
                'Enable', voronoiEnabled);
            astarEnabled = matlab.lang.OnOffSwitchState(strcmp(app.PlannerDropDown.Value, 'astar'));
            app.HeuristicTypeDropDown.Enable = astarEnabled;
            app.HeuristicWeightField.Enable = astarEnabled;
            app.TieBreakingDropDown.Enable = astarEnabled;
            app.AStarPanel.Visible = astarEnabled;
            app.VoronoiPanel.Visible = voronoiEnabled;
            heights = app.PlannerPanel.Parent.RowHeight;
            if strcmp(app.PlannerDropDown.Value, 'rrt'), heights{1} = 70; else, heights{1} = 140; end
            app.PlannerPanel.Parent.RowHeight = heights;
        end

        function scenario = collectScenario(app)
            scenario.plannerName = app.PlannerDropDown.Value;
            scenario.map.width = app.WidthField.Value;
            scenario.map.height = app.HeightField.Value;
            scenario.map.resolution = app.ResolutionField.Value;

            scenario.start2d = [
                app.StartXField.Value, ...
                app.StartYField.Value];

            scenario.goal2d = [
                app.GoalXField.Value, ...
                app.GoalYField.Value];

            scenario.safetyRadius = app.SafetyRadiusField.Value;

            scenario.obstacles = app.ObstaclesTable.Data;

            if isempty(scenario.obstacles)
                scenario.obstacles = zeros(0, 4);
            end

            scenario.rrt.stepSize = app.StepSizeField.Value;
            scenario.rrt.goalBias = app.GoalBiasField.Value;
            scenario.rrt.maxIter = round(app.MaxIterField.Value);
            scenario.rrt.goalThreshold = app.GoalThresholdField.Value;
            scenario.rrt.collisionCheckStep = app.CollisionStepField.Value;
            scenario.rrt.smoothingIterations = round(app.SmoothIterField.Value);
            scenario.rrt.densePathStep = app.DenseStepField.Value;
            scenario.astar.heuristicType = app.HeuristicTypeDropDown.Value;
            scenario.astar.heuristicWeight = app.HeuristicWeightField.Value;
            scenario.astar.tieBreaking = app.TieBreakingDropDown.Value;
            scenario.voronoi.clearanceWeight = app.ClearanceWeightField.Value;
            scenario.voronoi.clearanceExponent = app.ClearanceExponentField.Value;
            scenario.voronoi.connectionCount = round(app.ConnectionCountField.Value);

            scenario.quad = config.defaultQuadrotor();

            scenario.ctrl = config.defaultController();

            scenario.sim.dt = app.DtField.Value;
            scenario.sim.vRef = app.VRefField.Value;
            scenario.sim.zRef = app.ZRefField.Value;
            scenario.sim.maxTrackingError = app.MaxErrorField.Value;
        end

        function runPlanning(app)
            app.PlanningButton.Enable = 'off';
            cleanup = onCleanup(@() set(app.PlanningButton, 'Enable', 'on'));
            try
                app.StatusLabel.Text = 'Расчёт...';
                drawnow;

                scenario = app.collectScenario();
                app.CurrentResult = runPlannerExperiment(scenario, scenario.plannerName);

                viz.plotScenario( ...
                    app.PlotAxes, ...
                    scenario.map, ...
                    scenario.obstacles, ...
                    scenario.start2d, ...
                    scenario.goal2d, ...
                    app.CurrentResult.tree, ...
                    app.CurrentResult.path, ...
                    app.CurrentResult.simLog, ...
                    app.CurrentResult.rawPath, ...
                    app.CurrentResult.rrtPath, ...
                    app.CurrentResult.displayName);
                legend(app.PlotAxes, 'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 11);

                app.ResultTable.Data = [{app.CurrentResult.displayName}, table2cell(app.CurrentResult.summary)];

                app.plotSimulationGraphs();

                app.StatusLabel.Text = sprintf('%s: %s', ...
                    app.CurrentResult.displayName, app.CurrentResult.message);

            catch ME
                app.StatusLabel.Text = ['Ошибка: ', ME.message];
                uialert(app.UIFigure, ME.message, 'Ошибка расчёта');
            end
        end

        function plotSimulationGraphs(app)
            log = [];
            if ~isempty(app.CurrentResult)
                log = app.CurrentResult.simLog;
            end
            viz.plotTrackingError(app.ErrorComponentAxes, log);
            app.plotDerivativeGraphs();
        end

        function plotDerivativeGraphs(app)
        %PLOTDERIVATIVEGRAPHS Сравнивает производные траекторий только по x и y.
            for order = 1:4
                cla(app.DerivativeAxes(order));
                legend(app.DerivativeAxes(order), 'off');
            end
            if isempty(app.CurrentResult) || isempty(app.CurrentResult.simLog)
                return;
            end
            log = app.CurrentResult.simLog;
            actual = {log.firstDerivative, log.secondDerivative, log.thirdDerivative, log.fourthDerivative};
            reference = {log.vd, log.ad, log.jerk, log.snap};
            componentNames = {'x', 'y'};
            components = 1:2;
            if ~strcmp(app.DerivativeComponentDropDown.Value, 'all')
                components = find(strcmp(componentNames, app.DerivativeComponentDropDown.Value));
            end
            colors = lines(2);
            for order = 1:4
                ax = app.DerivativeAxes(order);
                hold(ax, 'on');
                for component = components
                    plot(ax, log.t, actual{order}(component, :), ...
                        'Color', colors(component, :), 'LineWidth', 1.3, ...
                        'DisplayName', sprintf('%s (ЛА)', componentNames{component}));
                    plot(ax, log.t, reference{order}(component, :), '--', ...
                        'Color', colors(component, :), 'LineWidth', 1.1, ...
                        'DisplayName', sprintf('%s (опорная)', componentNames{component}));
                end
                grid(ax, 'on');
                legend(ax, 'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 10);
                xlim(ax, [log.t(1), log.t(end)]);
                hold(ax, 'off');
            end
        end

        function addObstacle(app)
            data = app.ObstaclesTable.Data;

            if isempty(data)
                data = zeros(0, 4);
            end

            data = [data; 100, 100, 100, 100];

            app.ObstaclesTable.Data = data;
        end

        function deleteObstacle(app)
            data = app.ObstaclesTable.Data;

            if isempty(data)
                return;
            end

            data(end, :) = [];

            app.ObstaclesTable.Data = data;
        end

        function clearObstacles(app)
            app.ObstaclesTable.Data = zeros(0, 4);
        end

        function exportCsv(app)
            if isempty(app.CurrentResult)
                uialert(app.UIFigure, ...
                    'Сначала выполните расчёт.', ...
                    'Нет данных');
                return;
            end

            [file, path] = uiputfile('result.csv', ...
                'Сохранить результаты');

            if isequal(file, 0)
                return;
            end

            summary = addvars(app.CurrentResult.summary, string(app.CurrentResult.displayName), ...
                'Before', 1, 'NewVariableNames', 'Planner');
            writetable(summary, fullfile(path, file));

            app.StatusLabel.Text = 'Результаты сохранены';
        end

        function saveScenario(app)
            scenario = app.collectScenario();

            [file, path] = uiputfile('scenario.mat', ...
                'Сохранить сценарий');

            if isequal(file, 0)
                return;
            end

            save(fullfile(path, file), 'scenario');

            app.StatusLabel.Text = 'Сценарий сохранён';
        end

    end

end

function label = addLabel(parent, text)
label = uilabel(parent, ...
    'Text', text);

label.HorizontalAlignment = 'left';
end

function field = addNum(parent, value)
field = uieditfield(parent, ...
    'numeric', ...
    'Value', value);

if isprop(field, 'HorizontalAlignment')
    field.HorizontalAlignment = 'right';
end
end
