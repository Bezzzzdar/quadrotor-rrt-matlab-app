function tests = testVoronoi
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.TestData.oldPath = path;
    addpath(genpath(fullfile(root, 'src')));
end

function teardownOnce(testCase)
    path(testCase.TestData.oldPath);
end

function testBookEdgeCostAndClampedProjection(testCase)
    nodes = [0, 0; 10, 0];
    [cost, lengthValue, clearance] = voronoi.edgeCosts(nodes, [1, 2], [5, 2; 20, 0], 0.1, 0.9);
    verifyEqual(testCase, lengthValue, 10);
    verifyEqual(testCase, clearance, 2);
    verifyEqual(testCase, cost, 0.1 * 10 + 0.9 / 2, 'AbsTol', 1e-12);
    [~, ~, before] = voronoi.edgeCosts(nodes, [1, 2], [-3, 4], 0.1, 0.9);
    [~, ~, after] = voronoi.edgeCosts(nodes, [1, 2], [13, 4], 0.1, 0.9);
    verifyEqual(testCase, before, 5);
    verifyEqual(testCase, after, 5);
end

function testClearanceWeightChangesPreferredRoute(testCase)
    nodes = [0, 0; 5, 0; 10, 0; 5, 5];
    edges = [1, 2; 2, 3; 1, 4; 4, 3];
    sites = [5, 0.1];
    lowRiskWeight = voronoi.edgeCosts(nodes, edges, sites, 0.1, 0.001);
    highRiskWeight = voronoi.edgeCosts(nodes, edges, sites, 0.1, 10);
    verifyEqual(testCase, shortestpath(graph(edges(:, 1), edges(:, 2), lowRiskWeight), 1, 3), [1, 2, 3]);
    verifyEqual(testCase, shortestpath(graph(edges(:, 1), edges(:, 2), highRiskWeight), 1, 3), [1, 4, 3]);
end

function testExactCollisionBetweenSamples(testCase)
    map = struct('width', 10, 'height', 10, 'resolution', 1);
    verifyFalse(testCase, voronoi.isSegmentFree([0, 5], [10, 5], map, [4.1, 4, 0.1, 2], 0));
    verifyFalse(testCase, voronoi.isSegmentFree([0, 4], [10, 4], map, [4, 4, 2, 2], 0));
    verifyTrue(testCase, voronoi.isSegmentFree([0, 3], [10, 3], map, [4, 4, 2, 2], 0));
end

function testDefaultScenarioGraphAndRoute(testCase)
    scenario = config.defaultScenario();
    [route, roadmap] = voronoi.plan(scenario.start2d, scenario.goal2d, ...
        scenario.map, scenario.obstacles, scenario.safetyRadius, scenario.voronoi);
    verifyNotEmpty(testCase, route);
    verifyEqual(testCase, route(1, :), scenario.start2d);
    verifyEqual(testCase, route(end, :), scenario.goal2d);
    verifyLessThanOrEqual(testCase, size(roadmap.startConnection, 1), 3);
    verifyLessThanOrEqual(testCase, size(roadmap.goalConnection, 1), 3);
    verifyEqual(testCase, roadmap.weights, ...
        roadmap.k1 * roadmap.edgeLengths + roadmap.k2 ./ roadmap.edgeClearances, 'AbsTol', 1e-12);
    diagramEdgeCount = size(roadmap.edges, 1) - ...
        size(roadmap.startConnection, 1) - size(roadmap.goalConnection, 1);
    for index = unique(round(linspace(1, diagramEdgeCount, 30)))
        midpoint = mean(roadmap.nodes(roadmap.edges(index, :), :), 1);
        distances = sort(sqrt(sum((roadmap.sites - midpoint).^2, 2)));
        verifyEqual(testCase, distances(1), distances(2), 'AbsTol', 1e-6);
    end
    for index = 1:size(roadmap.edges, 1)
        edge = roadmap.nodes(roadmap.edges(index, :), :);
        assert(voronoi.isSegmentFree(edge(1, :), edge(2, :), ...
            scenario.map, scenario.obstacles, scenario.safetyRadius));
    end
    for index = 1:size(route, 1) - 1
        assert(geometry.isSegmentCollisionFree(route(index, :), route(index + 1, :), ...
            scenario.map, scenario.obstacles, scenario.safetyRadius, scenario.rrt.collisionCheckStep));
    end
    verifyEqual(testCase, sum(routeCosts(route, roadmap)), roadmap.pathCost, 'AbsTol', 1e-8);
end

function costs = routeCosts(route, roadmap)
    edges = [(1:size(route, 1) - 1)', (2:size(route, 1))'];
    costs = voronoi.edgeCosts(route, edges, roadmap.sites, roadmap.k1, roadmap.k2);
end

function testNearestThreeConnections(testCase)
    map = struct('width', 20, 'height', 20, 'resolution', 1);
    [route, roadmap] = voronoi.plan([3.2, 4.1], [16.3, 17.1], map, zeros(0, 4), 0);
    verifyNotEmpty(testCase, route);
    original = roadmap.nodes(1:end - 2, :);
    [~, nearest] = sort(sum((original - route(1, :)).^2, 2));
    verifyEqual(testCase, roadmap.startConnection(:, 2), nearest(1:3));
end

function testDisconnectedSpace(testCase)
    map = struct('width', 20, 'height', 20, 'resolution', 1);
    [route, roadmap] = voronoi.plan([3, 10], [17, 10], map, [9, 0, 2, 20], 0);
    verifyEmpty(testCase, route);
    verifyNotEmpty(testCase, roadmap.message);
    verifyEqual(testCase, roadmap.pathCost, Inf);
end

function testMinimumSiteCountOnCoarseEmptyMap(testCase)
    map = struct('width', 20, 'height', 20, 'resolution', 100);
    [~, ~, sites] = voronoi.buildRoadmap(map, zeros(0, 4), 0);
    verifyGreaterThanOrEqual(testCase, size(sites, 1), 10);
end
