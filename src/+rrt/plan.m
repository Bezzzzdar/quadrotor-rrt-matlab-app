function [path, tree] = plan(start2d, goal2d, map, obstacles, safetyRadius, params)
%PLAN Строит геометрический маршрут методом RRT.

nodes = start2d;
parents = 0;
path = [];

for iter = 1:params.maxIter
    if rand < params.goalBias
        sample = goal2d;
    else
        sample = [rand * map.width, rand * map.height];
    end

    nearestIndex = rrt.findNearestNode(nodes, sample);
    nearestNode = nodes(nearestIndex, :);
    newNode = rrt.steer(nearestNode, sample, params.stepSize);

    if ~geometry.isPointInsideMap(newNode, map)
        continue;
    end

    if geometry.isPointInObstacles(newNode, obstacles, safetyRadius)
        continue;
    end

    if ~geometry.isSegmentCollisionFree(nearestNode, newNode, map, obstacles, safetyRadius, params.collisionCheckStep)
        continue;
    end

    nodes = [nodes; newNode]; %#ok<AGROW>
    parents = [parents; nearestIndex]; %#ok<AGROW>
    newIndex = size(nodes, 1);

    if norm(newNode - goal2d) <= params.goalThreshold
        if geometry.isSegmentCollisionFree(newNode, goal2d, map, obstacles, safetyRadius, params.collisionCheckStep)
            nodes = [nodes; goal2d]; %#ok<AGROW>
            parents = [parents; newIndex]; %#ok<AGROW>
            path = rrt.restorePath(nodes, parents, size(nodes, 1));
            break;
        end
    end
end

tree.nodes = nodes;
tree.parents = parents;
end
