function [angle, times] = trajectoryCorners(segments)
%TRAJECTORYCORNERS Суммирует скачки направления касательной, включая остановки.
% Для негладких кривых полный поворот включает углы в вершинах.
% Литература: https://arxiv.org/pdf/math/0606007 (разделы 2 и 3).
    angle = 0;
    times = [];
    previousTangent = [];
    for index = 1:numel(segments)
        segment = segments(index);
        if segment.stationary
            continue;
        end
        startTangent = tangentAt(segment, 0, 1);
        if ~isempty(previousTangent)
            turn = atan2(norm(cross(previousTangent, startTangent)), dot(previousTangent, startTangent));
            if turn > 1e-7
                angle = angle + turn;
                times(end + 1) = segment.startTime; %#ok<AGROW>
            end
        end
        roots = segment.roots;
        interior = roots > 0 & roots < 1;
        reversing = interior & mod(segment.multiplicities, 2) == 1;
        angle = angle + pi * sum(reversing);
        times = [times, segment.startTime + roots(reversing) * ...
            (segment.endTime - segment.startTime)]; %#ok<AGROW>
        previousTangent = tangentAt(segment, 1, -1);
    end
end

function tangent = tangentAt(segment, parameter, side)
%TANGENTAT Вычисляет одностороннее направление движения в конце сегмента.
% Литература: https://arxiv.org/pdf/math/0606007 (раздел 3).
    factor = segment.factor;
    multiplicity = 0;
    index = find(abs(segment.roots - parameter) < 1e-9, 1);
    if ~isempty(index)
        multiplicity = segment.multiplicities(index);
        for power = 1:multiplicity
            quotient = factor(1:end - 1);
            for column = 2:numel(quotient)
                quotient(column) = factor(column) + parameter * quotient(column - 1);
            end
            factor = quotient;
        end
    end
    tangent = zeros(3, 1);
    for component = 1:3
        tangent(component) = polyval(segment.velocity(component, :), parameter);
    end
    tangent = tangent / norm(tangent) * sign(polyval(factor, parameter)) * side^multiplicity;
end
