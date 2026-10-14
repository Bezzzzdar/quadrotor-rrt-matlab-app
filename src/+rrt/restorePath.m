function path = restorePath(nodes, parents, goalIndex)
%RESTOREPATH Восстанавливает путь от цели к старту.

    ids = zeros(numel(parents), 1);
    count = 1;
    ids(count) = goalIndex;
    currentIndex = goalIndex;

    while parents(currentIndex) ~= 0
        currentIndex = parents(currentIndex);
        count = count + 1;
        ids(count) = currentIndex;
    end
    path = nodes(ids(count:-1:1), :);
end
