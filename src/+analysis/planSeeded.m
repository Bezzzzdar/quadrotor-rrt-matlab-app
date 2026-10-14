function planned = planSeeded(scenario, plannerName, seed)
%PLANSEEDED Сохраняет состояние генератора для последующей обработки RRT.
    rng(seed, 'twister');
    planned = planGeometricPath(scenario, plannerName);
    planned.rngState = rng;
end
