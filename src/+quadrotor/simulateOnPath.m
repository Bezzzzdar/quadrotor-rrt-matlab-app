function [log, traj] = simulateOnPath(path2d, quad, ctrl, sim, traj)
%SIMULATEONPATH Моделирует плоский полёт с идеальным удержанием z = zRef.
% Тяга компенсирует текущий наклон; дефицит вертикальной силы делает полёт нереализуемым.
% Ограничение высоты является допущением модели, а не результатом 3D-регулятора.

    validateattributes(sim.maxTrackingError, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    if nargin < 5
        traj = quadrotor.buildReferenceTrajectory(path2d, sim.vRef, sim.dt, sim.zRef);
    end
    N = length(traj.t);

    state.x = traj.xd(:, 1);
    state.v = traj.vd(:, 1);
    state.R = eye(3);
    state.Omega = zeros(3, 1);

    log.t = traj.t;
    log.x = zeros(3, N);
    log.xd = traj.xd;
    log.vd = traj.vd;
    log.ad = traj.ad;
    log.jerk = traj.jerk;
    log.snap = traj.snap;
    log.firstDerivative = zeros(3, N);
    log.secondDerivative = zeros(3, N);
    log.R = zeros(3, 3, N);
    log.Omega = zeros(3, N);
    log.f = zeros(1, N);
    log.M = zeros(3, N);
    log.motorThrusts = zeros(4, N);
    log.requestedMotorThrusts = zeros(4, N);
    log.motorSaturated = false(1, N);
    log.altitudeConstraintForce_N = zeros(1, N);
    log.altitudeAuthority = false(1, N);
    log.motionModel = 'planar_constant_altitude';
    log.trackingError = zeros(1, N);

    for k = 1:N
        xd = traj.xd(:, k);
        vd = traj.vd(:, k);
        ad = traj.ad(:, k);

        input = quadrotor.positionTrackingInput(state, xd, vd, ad, quad, ctrl);
        verticalProjection = dot(quad.e3, state.R * quad.e3);
        log.altitudeAuthority(k) = verticalProjection > 1e-6;
        if log.altitudeAuthority(k)
            input.f = quad.m * quad.g / verticalProjection;
        else
            % При потере вертикальной составляющей конечная тяга не удержит высоту.
            input.f = 4 * quad.fMaxPerMotor;
        end
        motorThrusts = quadrotor.thrustMomentToMotors(input.f, input.M, quad);
        log.requestedMotorThrusts(:, k) = motorThrusts;
        log.motorSaturated(k) = any(motorThrusts < quad.fMinPerMotor - 1e-9 | ...
            motorThrusts > quad.fMaxPerMotor + 1e-9);

        motorThrusts = min(max(motorThrusts, quad.fMinPerMotor), quad.fMaxPerMotor);
        [input.f, input.M] = quadrotor.motorsToThrustMoment(motorThrusts, quad);
        stateDot = quadrotor.dynamics(state, input, quad);
        log.altitudeConstraintForce_N(k) = -quad.m * stateDot.v(3);
        stateDot.x(3) = 0;
        stateDot.v(3) = 0;

        log.x(:, k) = state.x;
        log.firstDerivative(:, k) = stateDot.x;
        log.secondDerivative(:, k) = stateDot.v;
        log.R(:, :, k) = state.R;
        log.Omega(:, k) = state.Omega;
        log.f(k) = input.f;
        log.M(:, k) = input.M;
        log.motorThrusts(:, k) = motorThrusts;
        log.trackingError(k) = norm(state.x(1:2) - xd(1:2));

        if k < N
            dtStep = traj.t(k + 1) - traj.t(k);

            state.x = state.x + dtStep * stateDot.x;
            state.v = state.v + dtStep * stateDot.v;
            state.x(3) = sim.zRef;
            state.v(3) = 0;
            state.R = quadrotor.projectToSO3(state.R + dtStep * stateDot.R);
            state.Omega = state.Omega + dtStep * stateDot.Omega;
        end
    end

    log.thirdDerivative = differentiateSamples(log.secondDerivative, log.t);
    log.fourthDerivative = differentiateSamples(log.thirdDerivative, log.t);
    log.finalTrackingError = log.trackingError(end);
    log.maxTrackingError = max(log.trackingError);
    log.maxMotorThrust = max(log.motorThrusts(:));
    log.meanMotorThrust = mean(log.motorThrusts(:));
    log.dynamicFeasible = ~any(log.motorSaturated) && all(log.altitudeAuthority) && ...
        max(abs(log.altitudeConstraintForce_N)) <= 1e-8 * max(1, quad.m * quad.g) && ...
        log.maxTrackingError <= sim.maxTrackingError;
end

function derivative = differentiateSamples(values, time)
%DIFFERENTIATESAMPLES Вычисляет производные по времени.

    derivative = zeros(size(values));

    if numel(time) < 2
        return;
    end

    for row = 1:size(values, 1)
        derivative(row, :) = gradient(values(row, :), time);
    end
end
