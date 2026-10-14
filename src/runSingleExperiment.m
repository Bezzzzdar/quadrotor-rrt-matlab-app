function result = runSingleExperiment(scenario)
%RUNSINGLEEXPERIMENT Выполняет эксперимент RRT через общий конвейер.

    result = runPlannerExperiment(scenario, 'rrt');
    % Сохраняем прежний состав полей для приложения и анализа.
    result = rmfield(result, {'plannerName', 'displayName'});
end
