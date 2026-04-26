function lengthValue = calculatePathLength(path)
%CALCULATEPATHLENGTH Расчитывает длину пути.

if size(path, 1) < 2
    lengthValue = 0;
    return;
end

segmentLengths = sqrt(sum(diff(path).^2, 2));
lengthValue = sum(segmentLengths);
end
