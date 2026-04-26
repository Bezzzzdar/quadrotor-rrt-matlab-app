function input = positionTrackingInput(state, xd, vd, ad, quad, ctrl)
%POSITIONTRACKINGINPUT Формирует суммарную тягу f и момент M для слежения.
% Регулятор нужен для численной проверки динамической реализуемости пути.

ex = state.x - xd;
ev = state.v - vd;

aCmd = ad - ctrl.kpPos .* ex - ctrl.kdPos .* ev;
F = quad.m * (quad.g * quad.e3 - aCmd);
f = norm(F);

if f < 1e-9
    b3c = quad.e3;
else
    b3c = F / f;
end

b1d = [1; 0; 0];
if norm(cross(b3c, b1d)) < 1e-6
    b1d = [0; 1; 0];
end

b2c = cross(b3c, b1d);
b2c = b2c / norm(b2c);
b1c = cross(b2c, b3c);
Rc = [b1c, b2c, b3c];

eR = 0.5 * quadrotor.veeMap(Rc' * state.R - state.R' * Rc);
eOmega = state.Omega;
M = -ctrl.kR * eR - ctrl.kOmega * eOmega + cross(state.Omega, quad.J * state.Omega);

input.f = f;
input.M = M;
input.Rc = Rc;
input.ex = ex;
input.ev = ev;
end
