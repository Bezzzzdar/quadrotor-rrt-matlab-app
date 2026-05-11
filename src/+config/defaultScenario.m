function scenario = defaultScenario()
%DEFAULTSCENARIO Возвращает параметры сценария по умолчанию.

    scenario.map.width = 1000;
    scenario.map.height = 1000;
    scenario.map.resolution = 1;
    scenario.start2d = [50, 50];
    scenario.goal2d = [950, 950];
    scenario.safetyRadius = 5;
    scenario.obstacles = [
        % 180, 100,  80, 620;
        340, 300, 260,  80;
        % 660, 100, 100, 580;
        120, 820, 580,  70;
        % 820, 300,  80, 520;
        420, 520, 120, 180
    ];

    scenario.rrt.stepSize = 50;
    scenario.rrt.goalBias = 0.10;
    scenario.rrt.maxIter = 6000;
    scenario.rrt.goalThreshold = 50;
    scenario.rrt.collisionCheckStep = 1;
    scenario.rrt.smoothingIterations = 300;
    scenario.rrt.densePathStep = 1;

    scenario.sim.dt = 0.02;
    scenario.sim.vRef = 10;
    scenario.sim.zRef = 10;
    scenario.sim.maxTrackingError = 25;

    scenario.quad = config.defaultQuadrotor();
    scenario.ctrl = config.defaultController();
end
