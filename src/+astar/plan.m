function [path, search] = plan(start2d, goal2d, map, obstacles, safetyRadius, params)
%PLAN Строит маршрут алгоритмом A* на восьмисвязной сетке.
% Эвристики: https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
% Разрешение равенств: https://www.ijcai.org/proceedings/2018/655

    if nargin < 6, params = struct(); end
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
    validateattributes(map.width, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    validateattributes(map.height, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
    [freeGrid, xValues, yValues] = buildFreeGrid(map, obstacles, safetyRadius);

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
    geometry.validateScenario(map, start2d, goal2d, obstacles, safetyRadius);
    if isequal(start2d, goal2d)
        path = start2d;
        return;
    end
    startId = endpointNode(start2d, freeGrid, xValues, yValues, map, obstacles, safetyRadius);
    goalId = endpointNode(goal2d, freeGrid, xValues, yValues, map, obstacles, safetyRadius);
    if isempty(startId) || isempty(goalId)
        search.message = 'Нет безопасного подключения старта или цели к сетке.';
        return;
    end

    freeEdges = buildFreeEdges(xValues, yValues, obstacles, safetyRadius);
    [parents, visited, goalReached] = gridAStar(freeGrid, startId, goalId, xValues, yValues, ...
        heuristicType, heuristicWeight, tieBreaking, freeEdges);
    search.visitedIds = find(visited);
    search.parents = parents;
    if ~goalReached
        search.message = 'Маршрут на сетке не найден.';
        return;
    end

    path = restoreGridPath(parents, startId, goalId, xValues, yValues, size(freeGrid));
    if startId == goalId && geometry.isSegmentCollisionFree(start2d, goal2d, map, obstacles, safetyRadius)
        path = [start2d; goal2d];
    else
        if ~isequal(path(1, :), start2d), path = [start2d; path]; end
        if ~isequal(path(end, :), goal2d), path = [path; goal2d]; end
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

function id = endpointNode(point, freeGrid, xValues, yValues, map, obstacles, radius)
%ENDPOINTNODE Подключает точку к ближайшему видимому свободному узлу.
    [~, col] = min(abs(xValues - point(1)));
    [~, row] = min(abs(yValues - point(2)));
    id = sub2ind(size(freeGrid), row, col);
    if freeGrid(id) && geometry.isSegmentCollisionFree( ...
            point, [xValues(col), yValues(row)], map, obstacles, radius)
        return;
    end
    distances = (xValues - point(1)).^2 + (yValues' - point(2)).^2;
    distances(~freeGrid) = Inf;
    [values, order] = sort(distances(:));
    id = [];
    for candidate = 1:nnz(isfinite(values))
        [row, col] = ind2sub(size(freeGrid), order(candidate));
        if geometry.isSegmentCollisionFree(point, [xValues(col), yValues(row)], map, obstacles, radius)
            id = order(candidate);
            return;
        end
    end
end

function freeEdges = buildFreeEdges(xValues, yValues, obstacles, radius)
%BUILDFREEEDGES Заранее проверяет все рёбра, включая препятствия между узлами.
% Пакетная проверка интервалов исключает геометрические циклы внутри поиска.
% https://www.pbr-book.org/4ed/Shapes/Basic_Shape_Interface#RayndashBoundsIntersections
    height = numel(yValues);
    width = numel(xValues);
    offsets = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
    freeEdges = false(height*width, 8);
    for direction = 1:8
        dr = offsets(direction, 1);
        dc = offsets(direction, 2);
        rows = max(1, 1-dr):min(height, height-dr);
        cols = max(1, 1-dc):min(width, width-dc);
        links = true(numel(rows), numel(cols));
        for obstacle = 1:size(obstacles, 1)
            lower = obstacles(obstacle, 1:2) - radius;
            upper = obstacles(obstacle, 1:2) + obstacles(obstacle, 3:4) + radius;
            [xEnter, xLeave] = axisInterval(xValues(cols), xValues(cols+dc), lower(1), upper(1));
            [yEnter, yLeave] = axisInterval(yValues(rows)', yValues(rows+dr)', lower(2), upper(2));
            links = links & max(0, max(xEnter, yEnter)) > min(1, min(xLeave, yLeave));
        end
        ids = rows' + (cols-1)*height;
        freeEdges(ids(:), direction) = links(:);
    end
end

function [enter, leave] = axisInterval(a, b, lower, upper)
%AXISINTERVAL Возвращает интервал параметра внутри одной координатной полосы.
    enter = -inf(size(a));
    leave = inf(size(a));
    moving = b ~= a;
    first = (lower-a(moving))./(b(moving)-a(moving));
    second = (upper-a(moving))./(b(moving)-a(moving));
    enter(moving) = min(first, second);
    leave(moving) = max(first, second);
    outside = ~moving & (a < lower | a > upper);
    enter(outside) = Inf;
    leave(outside) = -Inf;
end

function [parents, visited, goalReached] = gridAStar(freeGrid, startId, goalId, xValues, yValues, heuristicType, heuristicWeight, tieBreaking, freeEdges)
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
            if ~freeEdges(currentId, k)
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
