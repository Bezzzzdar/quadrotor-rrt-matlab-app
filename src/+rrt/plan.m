function [path, tree] = plan(start2d, goal2d, map, obstacles, safetyRadius, params)
%PLAN Строит геометрический маршрут методом RRT.

    if isequal(start2d, goal2d)
        path = start2d;
        tree = struct('nodes', start2d, 'parents', 0);
        return;
    end
    nodes = zeros(params.maxIter + 2, 2);
    parents = zeros(params.maxIter + 2, 1);
    nodes(1, :) = start2d;
    nodeCount = 1;
    path = [];

    for iter = 1:params.maxIter
        if rand < params.goalBias
            sample = goal2d;
        else
            sample = [rand * map.width, rand * map.height];
        end

        nearestIndex = rrt.findNearestNode(nodes(1:nodeCount, :), sample);
        nearestNode = nodes(nearestIndex, :);
        newNode = rrt.steer(nearestNode, sample, params.stepSize);

        if ~geometry.isPointInsideMap(newNode, map)
            continue;
        end

        if geometry.isPointInObstacles(newNode, obstacles, safetyRadius)
            continue;
        end

        if ~geometry.isSegmentCollisionFree(nearestNode, newNode, map, obstacles, safetyRadius)
            continue;
        end

        nodeCount = nodeCount + 1;
        nodes(nodeCount, :) = newNode;
        parents(nodeCount) = nearestIndex;
        newIndex = nodeCount;

        if norm(newNode - goal2d) <= params.goalThreshold
            if geometry.isSegmentCollisionFree(newNode, goal2d, map, obstacles, safetyRadius)
                nodeCount = nodeCount + 1;
                nodes(nodeCount, :) = goal2d;
                parents(nodeCount) = newIndex;
                path = rrt.restorePath(nodes(1:nodeCount, :), parents(1:nodeCount), nodeCount);
                break;
            end
        end
    end

    tree.nodes = nodes(1:nodeCount, :);
    tree.parents = parents(1:nodeCount);
end
