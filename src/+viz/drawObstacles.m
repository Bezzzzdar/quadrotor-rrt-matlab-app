function drawObstacles(ax, obstacles)
%DRAWOBSTACLES Отображает прямоугольные препятствия.

    if isempty(obstacles)
        return;
    end

    for i = 1:size(obstacles, 1)
        rectangle(ax, 'Position', obstacles(i, :), ...
            'FaceColor', [0.35 0.35 0.35], ...
            'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
    end
end
