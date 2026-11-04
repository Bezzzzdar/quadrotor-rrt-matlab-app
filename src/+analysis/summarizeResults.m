function [tables, probabilities] = summarizeResults(samples, configurations)
%SUMMARIZERESULTS Формирует таблицы четырёх критериев и отдельные вероятности.
% Все моменты оцениваются только по допустимым minimum-snap траекториям.
% Дисперсия исправленная, с делителем n-1; при n<2 её оценка отсутствует.
% Источник: https://www.itl.nist.gov/div898/handbook/eda/section3/eda356.htm
% Вероятность равна числу допустимых траекторий, делённому на число всех попыток.

    criteria = ["Время планирования, с"; "Длина траектории, м"; ...
        "Максимальная кривизна, 1/м"; "Средняя кривизна, 1/м"];
    tables = struct();
    probabilities = struct();
    metricNames = {'PlanningTime_s', 'PathLength_m', 'MaxCurvature_1_m', 'MeanCurvature_1_m'};
    for index = 1:numel(configurations)
        selected = samples(samples.Configuration == string(configurations(index).Name), :);
        expectation = nan(4, 1);
        variance = nan(4, 1);
        for metric = 1:4
            values = selected.(metricNames{metric});
            values = values(selected.Solved & ~isnan(values));
            if ~isempty(values)
                expectation(metric) = mean(values);
                if numel(values) >= 2 && all(isfinite(values))
                    variance(metric) = var(values, 0);
                end
            end
        end
        planner = char(configurations(index).Planner);
        tables.(planner) = table(criteria, expectation, variance, ...
            'VariableNames', {'Criterion', 'Mean', 'Variance'});
        probabilities.(planner) = NaN;
        if height(selected) > 0
            probabilities.(planner) = sum(selected.Solved) / height(selected);
        end
    end
end
