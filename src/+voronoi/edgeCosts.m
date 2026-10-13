function [weights, lengths, clearances] = edgeCosts(nodes, edges, sites, k1, k2)
%EDGECOSTS Eq. (12.1): J = k1*length + k2/minimum distance to Q.
% The projection parameter is clamped to [0,1] as on book pp. 219-220.

    lengths = zeros(size(edges, 1), 1);
    clearances = zeros(size(edges, 1), 1);
    for index = 1:size(edges, 1)
        a = nodes(edges(index, 1), :);
        b = nodes(edges(index, 2), :);
        direction = b - a;
        squaredLength = sum(direction.^2);
        lengths(index) = sqrt(squaredLength);
        if squaredLength == 0
            closest = repmat(a, size(sites, 1), 1);
        else
            fraction = max(0, min(1, ((sites - a) * direction') / squaredLength));
            closest = a + fraction .* direction;
        end
        clearances(index) = min(sqrt(sum((sites - closest).^2, 2)));
    end
    weights = k1 * lengths + k2 ./ max(clearances, eps);
end
