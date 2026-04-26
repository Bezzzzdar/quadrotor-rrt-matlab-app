function [f, M] = motorsToThrustMoment(motorThrusts, quad)
%MOTORSTOTHRUSTMOMENT Вычисляет суммарную тягу и момент по тягам двигателей.

A = [
    1, 1, 1, 1;
    0, -quad.d, 0, quad.d;
    quad.d, 0, -quad.d, 0;
    -quad.cTauF, quad.cTauF, -quad.cTauF, quad.cTauF
];

fm = A * motorThrusts(:);
f = fm(1);
M = fm(2:4);
end
