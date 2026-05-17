function [log, traj] = simulateOnPath(path2d, quad, ctrl, sim)
%SIMULATEONPATH Моделирует движение квадрокоптера по minimum-snap маршруту.

    traj = quadrotor.buildReferenceTrajectory(path2d, sim.vRef, sim.dt, sim.zRef);
    traj = completeReferenceDerivatives(traj);
    N = length(traj.t);

    state.x = traj.xd(:, 1);
    state.v = traj.vd(:, 1);
    state.R = eye(3);
    state.Omega = zeros(3, 1);

    log.t = traj.t;
    log.x = zeros(3, N);
    log.v = zeros(3, N);
    log.xd = traj.xd;
    log.vd = traj.vd;
    log.ad = traj.ad;
    log.jerk = traj.jerk;
    log.snap = traj.snap;
    log.referenceFirstDerivative = traj.vd;
    log.referenceSecondDerivative = traj.ad;
    log.referenceThirdDerivative = traj.jerk;
    log.referenceFourthDerivative = traj.snap;
    log.firstDerivative = zeros(3, N);
    log.secondDerivative = zeros(3, N);
    log.thirdDerivative = zeros(3, N);
    log.fourthDerivative = zeros(3, N);
    log.R = zeros(3, 3, N);
    log.Omega = zeros(3, N);
    log.f = zeros(1, N);
    log.M = zeros(3, N);
    log.motorThrusts = zeros(4, N);
    log.trackingError = zeros(1, N);

    for k = 1:N
        xd = traj.xd(:, k);
        vd = traj.vd(:, k);
        ad = traj.ad(:, k);

        input = quadrotor.positionTrackingInput(state, xd, vd, ad, quad, ctrl);
        motorThrusts = quadrotor.thrustMomentToMotors(input.f, input.M, quad);

        motorThrusts = min(max(motorThrusts, quad.fMinPerMotor), quad.fMaxPerMotor);
        [input.f, input.M] = quadrotor.motorsToThrustMoment(motorThrusts, quad);
        stateDot = quadrotor.dynamics(state, input, quad);

        log.x(:, k) = state.x;
        log.v(:, k) = state.v;
        log.firstDerivative(:, k) = stateDot.x;
        log.secondDerivative(:, k) = stateDot.v;
        log.R(:, :, k) = state.R;
        log.Omega(:, k) = state.Omega;
        log.f(k) = input.f;
        log.M(:, k) = input.M;
        log.motorThrusts(:, k) = motorThrusts;
        log.trackingError(k) = norm(state.x - xd);

        if k < N
            dtStep = traj.t(k + 1) - traj.t(k);

            state.x = state.x + dtStep * stateDot.x;
            state.v = state.v + dtStep * stateDot.v;
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
    log.dynamicFeasible = log.maxMotorThrust <= quad.fMaxPerMotor + 1e-9 && ...
                        log.finalTrackingError <= sim.maxTrackingError;
end

function traj = completeReferenceDerivatives(traj)
%COMPLETEREFERENCEDERIVATIVES Добавляет недостающие производные траектории.

    if ~isfield(traj, 'vd') || isempty(traj.vd)
        traj.vd = differentiateSamples(traj.xd, traj.t);
    end

    if ~isfield(traj, 'ad') || isempty(traj.ad)
        traj.ad = differentiateSamples(traj.vd, traj.t);
    end

    if ~isfield(traj, 'jerk') || isempty(traj.jerk)
        traj.jerk = differentiateSamples(traj.ad, traj.t);
    end

    if ~isfield(traj, 'snap') || isempty(traj.snap)
        traj.snap = differentiateSamples(traj.jerk, traj.t);
    end
end

function derivative = differentiateSamples(values, time)
%DIFFERENTIATESAMPLES Вычисялет производные по времени

    derivative = zeros(size(values));

    if numel(time) < 2
        return;
    end

    for row = 1:size(values, 1)
        derivative(row, :) = gradient(values(row, :), time);
    end
end
