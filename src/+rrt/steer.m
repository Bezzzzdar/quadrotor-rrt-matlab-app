function newNode = steer(fromNode, toNode, stepSize)
%STEER Создаёт новую вершину на расстоянии stepSize.

    direction = toNode - fromNode;
    distance = norm(direction);

    if distance <= stepSize
        newNode = toNode;
    else
        newNode = fromNode + direction / distance * stepSize;
    end
end
