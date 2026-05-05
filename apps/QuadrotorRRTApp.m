classdef QuadrotorRRTApp < matlab.apps.AppBase
    %QUADROTORRRTAPP MATLAB-приложение для планирования траектории RRT.

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
        ErrorAxes matlab.ui.control.UIAxes
        MotorAxes matlab.ui.control.UIAxes
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
            app.UIFigure = uifigure( ...
                'Name', 'RRT-планирование траектории квадрокоптера', ...
                'Position', [100 100 100 100]);

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
            leftGrid.RowHeight = {'1x', 90};
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

            paramGrid = uigridlayout(paramPanel, [4 1]);
            paramGrid.RowHeight = {145, 175, 95, '1x'};
            paramGrid.ColumnWidth = {'1x'};
            paramGrid.Padding = [8 8 8 8];
            paramGrid.RowSpacing = 8;

            %% ===================== РАБОЧАЯ ОБЛАСТЬ =====================

            mapPanel = uipanel(paramGrid, ...
                'Title', 'Рабочая область');
            mapPanel.Layout.Row = 1;

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
                'Title', 'Параметры RRT');
            rrtPanel.Layout.Row = 2;

            rrtGrid = uigridlayout(rrtPanel, [4 4]);
            rrtGrid.RowHeight = {26, 26, 26, 26};
            rrtGrid.ColumnWidth = {105, '1x', 105, '1x'};
            rrtGrid.Padding = [8 6 8 6];
            rrtGrid.RowSpacing = 5;
            rrtGrid.ColumnSpacing = 10;

            addLabel(rrtGrid, 'Шаг, м');
            app.StepSizeField = addNum(rrtGrid, 50);

            addLabel(rrtGrid, 'Goal bias');
            app.GoalBiasField = addNum(rrtGrid, 0.1);

            addLabel(rrtGrid, 'Итерации');
            app.MaxIterField = addNum(rrtGrid, 6000);

            addLabel(rrtGrid, 'Порог цели');
            app.GoalThresholdField = addNum(rrtGrid, 50);

            addLabel(rrtGrid, 'Шаг проверки');
            app.CollisionStepField = addNum(rrtGrid, 1);

            addLabel(rrtGrid, 'Сглаживание');
            app.SmoothIterField = addNum(rrtGrid, 300);

            addLabel(rrtGrid, 'Шаг пути');
            app.DenseStepField = addNum(rrtGrid, 1);

            uilabel(rrtGrid, 'Text', '');
            uilabel(rrtGrid, 'Text', '');

            %% ===================== ПАРАМЕТРЫ СИМУЛЯЦИИ =====================

            simPanel = uipanel(paramGrid, ...
                'Title', 'Параметры симуляции');
            simPanel.Layout.Row = 3;

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
            obstaclePanel.Layout.Row = 4;

            obstacleGrid = uigridlayout(obstaclePanel, [2 1]);
            obstacleGrid.RowHeight = {'1x', 30};
            obstacleGrid.ColumnWidth = {'1x'};
            obstacleGrid.Padding = [8 6 8 6];
            obstacleGrid.RowSpacing = 5;

            app.ObstaclesTable = uitable(obstacleGrid);
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
            actionGrid.RowHeight = {30, 24};
            actionGrid.ColumnWidth = {'1.25x', '1x', '1.25x'};
            actionGrid.Padding = [8 6 8 6];
            actionGrid.RowSpacing = 4;
            actionGrid.ColumnSpacing = 8;

            uibutton(actionGrid, ...
                'Text', 'Построить маршрут', ...
                'ButtonPushedFcn', @(~,~) app.runPlanning());

            uibutton(actionGrid, ...
                'Text', 'Сохранить CSV', ...
                'ButtonPushedFcn', @(~,~) app.exportCsv());

            uibutton(actionGrid, ...
                'Text', 'Сохранить сценарий', ...
                'ButtonPushedFcn', @(~,~) app.saveScenario());

            app.StatusLabel = uilabel(actionGrid, ...
                'Text', 'Готово');

            app.StatusLabel.Layout.Row = 2;
            app.StatusLabel.Layout.Column = [1 3];

            %% ===================== ПРАВАЯ ПАНЕЛЬ =====================

            rightGrid = uigridlayout(app.RightPanel, [4 1]);
            rightGrid.RowHeight = {'2x', '1x', '1x', 130};
            rightGrid.ColumnWidth = {'1x'};
            rightGrid.Padding = [10 10 10 10];
            rightGrid.RowSpacing = 8;

            app.PlotAxes = uiaxes(rightGrid);
            title(app.PlotAxes, 'Карта и маршрут');
            xlabel(app.PlotAxes, 'X, м');
            ylabel(app.PlotAxes, 'Y, м');

            app.ErrorAxes = uiaxes(rightGrid);
            title(app.ErrorAxes, 'Ошибка слежения');
            xlabel(app.ErrorAxes, 'Время, с');
            ylabel(app.ErrorAxes, 'Ошибка, м');

            app.MotorAxes = uiaxes(rightGrid);
            title(app.MotorAxes, 'Тяги двигателей');
            xlabel(app.MotorAxes, 'Время, с');
            ylabel(app.MotorAxes, 'Тяга, Н');

            app.ResultTable = uitable(rightGrid);
        end

        function loadDefaultScenario(app)
            s = config.defaultScenario();

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
                []);
        end

        function scenario = collectScenario(app)
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

            scenario.quad = config.defaultQuadrotor();

            scenario.ctrl = config.defaultController();

            scenario.sim.dt = app.DtField.Value;
            scenario.sim.vRef = app.VRefField.Value;
            scenario.sim.zRef = app.ZRefField.Value;
            scenario.sim.maxTrackingError = app.MaxErrorField.Value;
        end

        function runPlanning(app)
            try
                app.StatusLabel.Text = 'Расчёт...';
                drawnow;

                scenario = app.collectScenario();
                app.CurrentResult = runSingleExperiment(scenario);

                viz.plotScenario( ...
                    app.PlotAxes, ...
                    scenario.map, ...
                    scenario.obstacles, ...
                    scenario.start2d, ...
                    scenario.goal2d, ...
                    app.CurrentResult.tree, ...
                    app.CurrentResult.path, ...
                    app.CurrentResult.simLog, ...
                    app.CurrentResult.rawPath);

                app.ResultTable.Data = table2cell(app.CurrentResult.summary);

                app.ResultTable.ColumnName = { ...
                    'Маршрут найден', ...
                    'Время расчёта, с', ...
                    'Длина пути, м', ...
                    'Макс. кривизна, 1/м', ...
                    'Средняя кривизна, 1/м', ...
                    'Динамически реализуем', ...
                    'Конечная ошибка, м', ...
                    'Макс. ошибка, м', ...
                    'Макс. тяга, Н', ...
                    'Средняя тяга, Н'};

                app.plotSimulationGraphs();

                app.StatusLabel.Text = app.CurrentResult.message;

            catch ME
                app.StatusLabel.Text = ['Ошибка: ', ME.message];
                uialert(app.UIFigure, ME.message, 'Ошибка расчёта');
            end
        end

        function plotSimulationGraphs(app)
            cla(app.ErrorAxes);
            cla(app.MotorAxes);

            if isempty(app.CurrentResult) || isempty(app.CurrentResult.simLog)
                return;
            end

            log = app.CurrentResult.simLog;

            plot(app.ErrorAxes, log.t, log.trackingError, ...
                'LineWidth', 2);

            grid(app.ErrorAxes, 'on');
            xlabel(app.ErrorAxes, 'Время, с');
            ylabel(app.ErrorAxes, 'Ошибка, м');
            title(app.ErrorAxes, 'Ошибка слежения динамической модели');

            plot(app.MotorAxes, log.t, log.motorThrusts', ...
                'LineWidth', 1.2);

            grid(app.MotorAxes, 'on');
            xlabel(app.MotorAxes, 'Время, с');
            ylabel(app.MotorAxes, 'Тяга, Н');
            title(app.MotorAxes, 'Тяги двигателей');
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

            writetable(app.CurrentResult.summary, fullfile(path, file));

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
