function densePath = interpolatePath(path, step)
%INTERPOLATEPATH Уплотняет путь точками с заданным шагом.

    densePath = [];
    if isempty(path)
        return;
    end

    validateattributes(step, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    deltas = diff(path, 1, 1);
    counts = max(2, ceil(sqrt(sum(deltas.^2, 2)) / step));
    densePath = zeros(sum(counts) + 1, size(path, 2));
    offset = 0;

    for i = 1:size(path, 1) - 1
        p1 = path(i, :);
        n = counts(i);
        densePath(offset + (1:n), :) = p1 + ((0:n-1)' / n) .* deltas(i, :);
        offset = offset + n;
    end

    densePath(end, :) = path(end, :);
end
