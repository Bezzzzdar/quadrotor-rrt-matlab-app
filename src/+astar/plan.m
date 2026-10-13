function [path, search] = plan(start2d, goal2d, map, obstacles, safetyRadius, params)
%PLAN Builds a grid path with A* search.

    if ~isfield(map, 'resolution') || map.resolution <= 0
        error('Map resolution must be positive.');
    end

    resolution = map.resolution;
    [freeGrid, xValues, yValues] = buildFreeGrid(map, obstacles, safetyRadius);

    [startRow, startCol] = pointToGrid(start2d, xValues, yValues, resolution);
    [goalRow, goalCol] = pointToGrid(goal2d, xValues, yValues, resolution);

    path = [];
    search = struct();
    search.freeGrid = freeGrid;
    search.xValues = xValues;
    search.yValues = yValues;
    search.visitedIds = [];
    search.parents = [];

    if ~freeGrid(startRow, startCol) || ~freeGrid(goalRow, goalCol)
        return;
    end

    startId = sub2ind(size(freeGrid), startRow, startCol);
    goalId = sub2ind(size(freeGrid), goalRow, goalCol);

    [parents, visited, goalReached] = gridAStar( ...
        freeGrid, startId, goalId, xValues, yValues);

    search.visitedIds = find(visited);
    search.parents = parents;

    if ~goalReached
        return;
    end

    path = restoreGridPath(parents, startId, goalId, xValues, yValues, size(freeGrid));

    if norm(path(1, :) - start2d) > eps
        path(1, :) = start2d;
    end

    if norm(path(end, :) - goal2d) > eps
        path(end, :) = goal2d;
    end

    if isfield(params, 'collisionCheckStep')
        path = removeInvalidShortcuts(path, map, obstacles, safetyRadius, params.collisionCheckStep);
    end
end

function [freeGrid, xValues, yValues] = buildFreeGrid(map, obstacles, safetyRadius)
%BUILDFREEGRID Converts rectangular obstacles to an inflated occupancy grid.

    resolution = map.resolution;
    xValues = 0:resolution:map.width;
    yValues = 0:resolution:map.height;

    if xValues(end) < map.width
        xValues = [xValues, map.width];
    end

    if yValues(end) < map.height
        yValues = [yValues, map.height];
    end

    blocked = false(numel(yValues), numel(xValues));

    for i = 1:size(obstacles, 1)
        xMin = obstacles(i, 1) - safetyRadius;
        xMax = obstacles(i, 1) + obstacles(i, 3) + safetyRadius;
        yMin = obstacles(i, 2) - safetyRadius;
        yMax = obstacles(i, 2) + obstacles(i, 4) + safetyRadius;

        colMask = xValues >= xMin & xValues <= xMax;
        rowMask = yValues >= yMin & yValues <= yMax;
        blocked(rowMask, colMask) = true;
    end

    freeGrid = ~blocked;
end

function [row, col] = pointToGrid(point, xValues, yValues, resolution)
%POINTTOGRID Maps a world point to the nearest grid cell.

    col = round(point(1) / resolution) + 1;
    row = round(point(2) / resolution) + 1;

    col = min(max(col, 1), numel(xValues));
    row = min(max(row, 1), numel(yValues));
end

function [parents, visited, goalReached] = gridAStar(freeGrid, startId, goalId, xValues, yValues)
%GRIDASTAR Searches an eight-connected occupancy grid.

    gridSize = size(freeGrid);
    nodeCount = numel(freeGrid);

    parents = zeros(nodeCount, 1, 'uint32');
    visited = false(nodeCount, 1);
    gScore = inf(nodeCount, 1);

    [goalRow, goalCol] = ind2sub(gridSize, goalId);
    goalPoint = [xValues(goalCol), yValues(goalRow)];

    gScore(startId) = 0;

    heap.ids = zeros(1024, 1, 'uint32');
    heap.priorities = inf(1024, 1);
    heap.count = 0;
    heap = heapPush(heap, startId, heuristic(startId, gridSize, xValues, yValues, goalPoint));

    neighborOffsets = [
        -1, -1;
        -1,  0;
        -1,  1;
         0, -1;
         0,  1;
         1, -1;
         1,  0;
         1,  1
    ];

    goalReached = false;

    while heap.count > 0
        [heap, currentId] = heapPop(heap);
        currentId = double(currentId);

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

            if dRow ~= 0 && dCol ~= 0
                if ~freeGrid(currentRow, nextCol) || ~freeGrid(nextRow, currentCol)
                    continue;
                end
            end

            nextId = sub2ind(gridSize, nextRow, nextCol);

            if visited(nextId)
                continue;
            end

            stepCost = hypot( ...
                xValues(nextCol) - xValues(currentCol), ...
                yValues(nextRow) - yValues(currentRow));
            tentativeG = gScore(currentId) + stepCost;

            if tentativeG < gScore(nextId)
                parents(nextId) = uint32(currentId);
                gScore(nextId) = tentativeG;
                priority = tentativeG + heuristic(nextId, gridSize, xValues, yValues, goalPoint);
                heap = heapPush(heap, nextId, priority);
            end
        end
    end
end

function h = heuristic(nodeId, gridSize, xValues, yValues, goalPoint)
%HEURISTIC Returns Euclidean distance to the goal.

    [row, col] = ind2sub(gridSize, nodeId);
    h = hypot(xValues(col) - goalPoint(1), yValues(row) - goalPoint(2));
end

function path = restoreGridPath(parents, startId, goalId, xValues, yValues, gridSize)
%RESTOREGRIDPATH Converts parent links to world coordinates.

    ids = goalId;
    currentId = goalId;

    while currentId ~= startId
        currentId = double(parents(currentId));

        if currentId == 0
            path = [];
            return;
        end

        ids = [currentId; ids]; %#ok<AGROW>
    end

    path = zeros(numel(ids), 2);

    for i = 1:numel(ids)
        [row, col] = ind2sub(gridSize, ids(i));
        path(i, :) = [xValues(col), yValues(row)];
    end
end

function path = removeInvalidShortcuts(path, map, obstacles, safetyRadius, checkStep)
%REMOVEINVALIDSHORTCUTS Keeps only adjacent grid steps if endpoint snapping is unsafe.

    if size(path, 1) < 2
        return;
    end

    if ~geometry.isSegmentCollisionFree(path(1, :), path(2, :), map, obstacles, safetyRadius, checkStep)
        path(1, :) = path(2, :);
    end

    lastIndex = size(path, 1);
    if ~geometry.isSegmentCollisionFree(path(lastIndex - 1, :), path(lastIndex, :), map, obstacles, safetyRadius, checkStep)
        path(lastIndex, :) = path(lastIndex - 1, :);
    end
end

function heap = heapPush(heap, id, priority)
%HEAPPUSH Inserts one item into a binary min-heap.

    heap.count = heap.count + 1;

    if heap.count > numel(heap.ids)
        heap.ids = [heap.ids; zeros(numel(heap.ids), 1, 'uint32')];
        heap.priorities = [heap.priorities; inf(numel(heap.priorities), 1)];
    end

    index = heap.count;
    heap.ids(index) = uint32(id);
    heap.priorities(index) = priority;

    while index > 1
        parentIndex = floor(index / 2);

        if heap.priorities(parentIndex) <= heap.priorities(index)
            break;
        end

        heap = heapSwap(heap, index, parentIndex);
        index = parentIndex;
    end
end

function [heap, id] = heapPop(heap)
%HEAPPOP Removes the item with the lowest priority.

    id = heap.ids(1);
    heap.ids(1) = heap.ids(heap.count);
    heap.priorities(1) = heap.priorities(heap.count);
    heap.count = heap.count - 1;

    index = 1;

    while true
        leftIndex = index * 2;
        rightIndex = leftIndex + 1;
        smallestIndex = index;

        if leftIndex <= heap.count && heap.priorities(leftIndex) < heap.priorities(smallestIndex)
            smallestIndex = leftIndex;
        end

        if rightIndex <= heap.count && heap.priorities(rightIndex) < heap.priorities(smallestIndex)
            smallestIndex = rightIndex;
        end

        if smallestIndex == index
            break;
        end

        heap = heapSwap(heap, index, smallestIndex);
        index = smallestIndex;
    end
end

function heap = heapSwap(heap, firstIndex, secondIndex)
%HEAPSWAP Swaps two heap entries.

    id = heap.ids(firstIndex);
    priority = heap.priorities(firstIndex);

    heap.ids(firstIndex) = heap.ids(secondIndex);
    heap.priorities(firstIndex) = heap.priorities(secondIndex);

    heap.ids(secondIndex) = id;
    heap.priorities(secondIndex) = priority;
end
