function [path, search] = plan(start2d, goal2d, map, obstacles, safetyRadius, params)
%PLAN Строит маршрут алгоритмом A* на восьмисвязной сетке.
% Эвристики: https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
% Разрешение равенств: https://www.ijcai.org/proceedings/2018/655

    heuristicType = "euclidean";
    heuristicWeight = 1;
    tieBreaking = "largerg";
    if isfield(params, 'heuristicType'), heuristicType = lower(string(params.heuristicType)); end
    if isfield(params, 'heuristicWeight'), heuristicWeight = params.heuristicWeight; end
    if isfield(params, 'tieBreaking'), tieBreaking = lower(string(params.tieBreaking)); end
    if ~isscalar(heuristicType) || ~ismember(heuristicType, ["euclidean", "octile"])
        error('astar:InvalidHeuristic', 'Эвристика должна быть euclidean или octile.');
    end
    if ~isscalar(tieBreaking) || ~ismember(tieBreaking, ["largerg", "smallerg"])
        error('astar:InvalidTieBreaking', 'Правило равенств должно быть largerg или smallerg.');
    end
    validateattributes(heuristicWeight, {'numeric'}, {'scalar', 'real', 'finite', 'positive', '<=', 1});

    validateattributes(map.resolution, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    [freeGrid, xValues, yValues] = buildFreeGrid(map, obstacles, safetyRadius);
    [startRow, startCol] = pointToGrid(start2d, xValues, yValues);
    [goalRow, goalCol] = pointToGrid(goal2d, xValues, yValues);

    path = [];
    search = struct('freeGrid', freeGrid, 'xValues', xValues, 'yValues', yValues, ...
        'visitedIds', [], 'parents', [], 'message', '', ...
        'heuristicType', heuristicType, 'heuristicWeight', heuristicWeight, 'tieBreaking', tieBreaking);
    if ~geometry.isPointInsideMap(start2d, map) || ...
            ~geometry.isPointInsideMap(goal2d, map) || ...
            geometry.isPointInObstacles(start2d, obstacles, safetyRadius) || ...
            geometry.isPointInObstacles(goal2d, obstacles, safetyRadius)
        search.message = 'Старт или цель находятся вне свободного пространства.';
        return;
    end
    if ~freeGrid(startRow, startCol) || ~freeGrid(goalRow, goalCol)
        search.message = 'Ближайший к старту или цели узел сетки занят.';
        return;
    end

    startId = sub2ind(size(freeGrid), startRow, startCol);
    goalId = sub2ind(size(freeGrid), goalRow, goalCol);
    [parents, visited, goalReached] = gridAStar(freeGrid, startId, goalId, xValues, yValues, ...
        heuristicType, heuristicWeight, tieBreaking);
    search.visitedIds = find(visited);
    search.parents = parents;
    if ~goalReached
        search.message = 'Маршрут на сетке не найден.';
        return;
    end

    path = restoreGridPath(parents, startId, goalId, xValues, yValues, size(freeGrid));
    if size(path, 1) == 1
        path = [start2d; goal2d];
    else
        path(1, :) = start2d;
        path(end, :) = goal2d;
    end
    checkStep = map.resolution;
    if isfield(params, 'collisionCheckStep')
        checkStep = params.collisionCheckStep;
    end
    if ~geometry.isSegmentCollisionFree(path(1, :), path(2, :), ...
            map, obstacles, safetyRadius, checkStep) || ...
            ~geometry.isSegmentCollisionFree(path(end-1, :), path(end, :), ...
            map, obstacles, safetyRadius, checkStep)
        path = [];
        search.message = 'Нет безопасного подключения старта или цели к сетке.';
    end
end

function [freeGrid, xValues, yValues] = buildFreeGrid(map, obstacles, safetyRadius)
%BUILDFREEGRID Расширяет препятствия и формирует сетку свободных узлов.
    xValues = 0:map.resolution:map.width;
    yValues = 0:map.resolution:map.height;
    if xValues(end) < map.width
        xValues(end+1) = map.width;
    end
    if yValues(end) < map.height
        yValues(end+1) = map.height;
    end
    blocked = false(numel(yValues), numel(xValues));
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        cols = xValues >= obstacle(1) - safetyRadius & ...
            xValues <= obstacle(1) + obstacle(3) + safetyRadius;
        rows = yValues >= obstacle(2) - safetyRadius & ...
            yValues <= obstacle(2) + obstacle(4) + safetyRadius;
        blocked(rows, cols) = true;
    end
    freeGrid = ~blocked;
end

function [row, col] = pointToGrid(point, xValues, yValues)
%POINTTOGRID Учитывает укороченный шаг у границы карты.
    [~, col] = min(abs(xValues - point(1)));
    [~, row] = min(abs(yValues - point(2)));
end

function [parents, visited, goalReached] = gridAStar(freeGrid, startId, goalId, xValues, yValues, heuristicType, heuristicWeight, tieBreaking)
%GRIDASTAR Выполняет поиск без срезания углов занятых клеток.
    gridSize = size(freeGrid);
    nodeCount = numel(freeGrid);
    parents = zeros(nodeCount, 1, 'uint32');
    visited = false(nodeCount, 1);
    gScore = inf(nodeCount, 1);
    [goalRow, goalCol] = ind2sub(gridSize, goalId);
    goalPoint = [xValues(goalCol), yValues(goalRow)];
    gScore(startId) = 0;
    useOctile = heuristicType == "octile";
    diagonalExtra = sqrt(2) - 1;
    octileScale = 1;
    if useOctile
        % Нормировка сохраняет согласованность при укороченном шаге у границы.
        [dx, dy] = ndgrid(unique(diff(xValues)), unique(diff(yValues)));
        octileScale = min(hypot(dx, dy) ./ ...
            (max(dx, dy) + diagonalExtra * min(dx, dy)), [], 'all');
    end
    tieSign = 1;
    if tieBreaking == "largerg", tieSign = -1; end

    % Вложенные функции изменяют общий буфер без передачи структуры очереди.
    heapIds = zeros(1024, 1, 'uint32');
    heapPriorities = inf(1024, 1);
    heapTies = zeros(1024, 1);
    heapCount = 0;
    [startRow, startCol] = ind2sub(gridSize, startId);
    dx = abs(xValues(startCol) - goalPoint(1));
    dy = abs(yValues(startRow) - goalPoint(2));
    if useOctile
        h = octileScale * (max(dx, dy) + diagonalExtra * min(dx, dy));
    else
        h = hypot(dx, dy);
    end
    heapPush(startId, heuristicWeight * h, 0);
    neighborOffsets = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
    goalReached = false;
    while heapCount > 0
        currentId = double(heapPop());
        if visited(currentId)
            continue;
        end
        visited(currentId) = true;
        if currentId == goalId
            goalReached = true;
            return;
        end
        [currentRow, currentCol] = ind2sub(gridSize, currentId);
        for k = 1:size(neighborOffsets, 1)
            dRow = neighborOffsets(k, 1);
            dCol = neighborOffsets(k, 2);
            nextRow = currentRow + dRow;
            nextCol = currentCol + dCol;
            if nextRow < 1 || nextRow > gridSize(1) || nextCol < 1 || nextCol > gridSize(2)
                continue;
            end
            if ~freeGrid(nextRow, nextCol)
                continue;
            end
            if dRow ~= 0 && dCol ~= 0 && ...
                    (~freeGrid(currentRow, nextCol) || ~freeGrid(nextRow, currentCol))
                continue;
            end
            nextId = nextRow + (nextCol - 1) * gridSize(1);
            if visited(nextId)
                continue;
            end
            stepCost = hypot(xValues(nextCol) - xValues(currentCol), ...
                yValues(nextRow) - yValues(currentRow));
            tentativeG = gScore(currentId) + stepCost;
            if tentativeG < gScore(nextId)
                parents(nextId) = uint32(currentId);
                gScore(nextId) = tentativeG;
                dx = abs(xValues(nextCol) - goalPoint(1));
                dy = abs(yValues(nextRow) - goalPoint(2));
                if useOctile
                    h = octileScale * (max(dx, dy) + diagonalExtra * min(dx, dy));
                else
                    h = hypot(dx, dy);
                end
                priority = tentativeG + heuristicWeight * h;
                heapPush(nextId, priority, tieSign * tentativeG);
            end
        end
    end

    function heapPush(id, priority, tie)
    %HEAPPUSH Перемещает записи вверх без попарных перестановок.
        heapCount = heapCount + 1;
        if heapCount > numel(heapIds)
            capacity = 2 * numel(heapIds);
            heapIds(capacity) = uint32(0);
            heapPriorities(capacity) = Inf;
            heapTies(capacity) = 0;
        end
        index = heapCount;
        while index > 1
            parentIndex = floor(index / 2);
            if heapPriorities(parentIndex) < priority || ...
                    (heapPriorities(parentIndex) == priority && heapTies(parentIndex) <= tie)
                break;
            end
            heapIds(index) = heapIds(parentIndex);
            heapPriorities(index) = heapPriorities(parentIndex);
            heapTies(index) = heapTies(parentIndex);
            index = parentIndex;
        end
        heapIds(index) = uint32(id);
        heapPriorities(index) = priority;
        heapTies(index) = tie;
    end

    function id = heapPop()
    %HEAPPOP Сравнивает сначала f, а при точном равенстве - выбранный знак g.
        id = heapIds(1);
        lastId = heapIds(heapCount);
        lastPriority = heapPriorities(heapCount);
        lastTie = heapTies(heapCount);
        heapCount = heapCount - 1;
        if heapCount == 0
            return;
        end
        index = 1;
        while 2 * index <= heapCount
            child = 2 * index;
            if child < heapCount && (heapPriorities(child+1) < heapPriorities(child) || ...
                    (heapPriorities(child+1) == heapPriorities(child) && heapTies(child+1) < heapTies(child)))
                child = child + 1;
            end
            if heapPriorities(child) > lastPriority || ...
                    (heapPriorities(child) == lastPriority && heapTies(child) >= lastTie)
                break;
            end
            heapIds(index) = heapIds(child);
            heapPriorities(index) = heapPriorities(child);
            heapTies(index) = heapTies(child);
            index = child;
        end
        heapIds(index) = lastId;
        heapPriorities(index) = lastPriority;
        heapTies(index) = lastTie;
    end
end

function path = restoreGridPath(parents, startId, goalId, xValues, yValues, gridSize)
%RESTOREGRIDPATH Восстанавливает координаты без наращивания массива.
    count = 1;
    currentId = goalId;
    while currentId ~= startId
        currentId = double(parents(currentId));
        count = count + 1;
    end
    ids = zeros(count, 1);
    currentId = goalId;
    for i = count:-1:1
        ids(i) = currentId;
        currentId = double(parents(currentId));
    end
    [rows, cols] = ind2sub(gridSize, ids);
    path = [reshape(xValues(cols), [], 1), reshape(yValues(rows), [], 1)];
end
