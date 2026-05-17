function densePath = interpolatePath(path, step)
%INTERPOLATEPATH Уплотняет путь точками с заданным шагом.

    densePath = [];
    if isempty(path)
        return;
    end

    for i = 1:size(path, 1) - 1
        p1 = path(i, :);
        p2 = path(i + 1, :);
        segmentLength = norm(p2 - p1);
        n = max(2, ceil(segmentLength / step));

        for k = 0:n-1
            t = k / n;
            densePath = [densePath; p1 + t * (p2 - p1)]; %#ok<AGROW>
        end
    end

    densePath = [densePath; path(end, :)];
end
