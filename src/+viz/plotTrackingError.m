function plotTrackingError(ax, log)
%PLOTTRACKINGERROR Показывает отклонения от minimum-snap только по x и y.
    cla(ax);
    legend(ax, 'off');
    if ~isempty(log)
        deviation = log.x(1:2, :) - log.xd(1:2, :);
        tag = ax.Tag;
        curves = plot(ax, log.t, deviation', 'LineWidth', 1.3);
        ax.Tag = tag;
        set(curves(1), 'DisplayName', 'e_{x}');
        set(curves(2), 'DisplayName', 'e_{z}');
        legend(ax, 'Location', 'northeast');
        xlim(ax, [log.t(1), log.t(end)]);
    end
    grid(ax, 'on');
    xlabel(ax, 't, с');
    ylabel(ax, 'e, м');
    title(ax, 'Ошибки слежения по x и z');
end
