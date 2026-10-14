function [weights, lengths, clearances] = edgeCosts(nodes, edges, sites, k1, k2, clearanceExponent)
%EDGECOSTS Вычисляет стоимость ребра J = k1*длина + k2/расстояние до Q.
% Проекция ограничена [0,1]: Малые БПЛА. Теория и практика (2015), с. 219-220.
% p<>1 - расширение исходного штрафа; длины нормированы на неизменный 1 метр.

    if nargin < 6, clearanceExponent = 1; end
    validateattributes(clearanceExponent, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});

    lengths = zeros(size(edges, 1), 1);
    clearances = zeros(size(edges, 1), 1);
    for index = 1:size(edges, 1)
        a = nodes(edges(index, 1), :);
        b = nodes(edges(index, 2), :);
        direction = b - a;
        squaredLength = sum(direction.^2);
        lengths(index) = sqrt(squaredLength);
        if squaredLength == 0
            closest = a;
        else
            fraction = max(0, min(1, ((sites - a) * direction') / squaredLength));
            closest = a + fraction .* direction;
        end
        clearances(index) = sqrt(min(sum((sites - closest).^2, 2)));
    end
    weights = k1 * lengths + k2 ./ max(clearances, eps).^clearanceExponent;
end
