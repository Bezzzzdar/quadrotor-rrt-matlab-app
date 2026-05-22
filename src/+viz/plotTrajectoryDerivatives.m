function plotTrajectoryDerivatives(log)
%PLOTTRAJECTORYDERIVATIVES Отображает графики первых четырёх производных.

    if isempty(log)
        return;
    end

    derivatives = { ...
        log.firstDerivative, ...
        log.secondDerivative, ...
        log.thirdDerivative, ...
        log.fourthDerivative
    };

    titles = { ...
        '1-я проиводная', ...
        '2-я проиводная', ...
        '3-я проиводная', ...
        '4-я проиводная'
    };

    yLabels = { ...
        'v, м/c', ...
        'a, м/c^2', ...
        'j, м/c^3', ...
        's, м/c^4'
    };

    componentNames = {'x', 'z'};

    figure('Name', 'Производные фактической траектории ЛА');

    for i = 1:numel(derivatives)
        subplot(numel(derivatives), 1, i);
        plot(log.t, derivatives{i}(1:2, :)', 'LineWidth', 1.5);
        grid on;
        grid minor
        xlabel('t, с');
        ylabel(yLabels{i});
        title(titles{i});
        legend(componentNames, 'Location', 'northeast');
        set(findall(gcf,'-property','FontName'), 'FontName', 'Times')
        set(findall(gcf,'-property','FontSize'), 'FontSize', 14)
        set(findall(gcf,'-property','Interpreter'), 'Interpreter', 'tex')
    end
end
