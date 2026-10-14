function [planned, status, message, workerTime, errorID] = runTimedPlan(pool, scenario, plannerName, seed, timeLimit)
%RUNTIMEDPLAN Ограничивает время задачи планирования, без очереди и прогрева.
% Бюджет задачи включает служебные расходы работника; calculationTime - только поиск.
% https://www.mathworks.com/help/matlab/ref/parallel.future.wait.html
% https://www.mathworks.com/help/matlab/ref/parallel.future.cancel.html
    future = parfeval(pool, @analysis.planSeeded, 1, scenario, plannerName, seed);
    cleanup = onCleanup(@() cancel(future));
    planned = [];
    status = "error";
    message = "";
    errorID = "";
    if ~wait(future, 'running', 30)
        cancelAndDrain(pool, future);
        error('comparison:QueueTimeout', 'Задача не начала выполнение за 30 секунд.');
    end
    remaining = timeLimit - seconds(future.RunningDuration);
    finished = strcmp(future.State, 'finished');
    if ~finished && remaining > 0
        finished = wait(future, 'finished', remaining);
    end
    workerTime = seconds(future.RunningDuration);
    if ~finished || workerTime > timeLimit
        cancelAndDrain(pool, future);
        status = "timeout";
        message = "Превышен общий лимит времени задачи планирования.";
        return;
    end
    try
        planned = fetchOutputs(future);
        status = "completed";
    catch exception
        message = string(exception.message);
        errorID = string(exception.identifier);
    end
end

function cancelAndDrain(pool, future)
%CANCELANDDRAIN Барьер не допускает следующего замера до освобождения работников.
% Некоторые встроенные операции не прерываются мгновенно; пул пользователя не удаляется.
    cancel(future);
    barrier = parfevalOnAll(pool, @pause, 0, 0);
    cleanup = onCleanup(@() cancel(barrier));
    if ~wait(barrier, 'finished', 30)
        error('comparison:CancellationPending', ...
            'Работник не освободился после отмены. Сравнение остановлено, пул не удалён.');
    end
    fetchOutputs(barrier);
end
