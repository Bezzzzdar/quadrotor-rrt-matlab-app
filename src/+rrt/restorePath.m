function path = restorePath(nodes, parents, goalIndex)
%RESTOREPATH Восстанавливает путь от цели к старту.

    path = nodes(goalIndex, :);
    currentIndex = goalIndex;

    while parents(currentIndex) ~= 0
        currentIndex = parents(currentIndex);
        path = [nodes(currentIndex, :); path]; %#ok<AGROW>
    end
end
