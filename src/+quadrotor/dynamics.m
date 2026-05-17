function stateDot = dynamics(state, input, quad)
%DYNAMICS Динамическая модель квадрокоптера на SE(3).

    R = state.R;
    Omega = state.Omega;
    f = input.f;
    M = input.M;

    stateDot.x = state.v;
    stateDot.v = quad.g * quad.e3 - (f / quad.m) * R * quad.e3;
    stateDot.R = R * quadrotor.hatMap(Omega);
    stateDot.Omega = quad.J \ (M - cross(Omega, quad.J * Omega));
end
