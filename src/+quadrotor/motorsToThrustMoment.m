function [f, M] = motorsToThrustMoment(motorThrusts, quad)
%MOTORSTOTHRUSTMOMENT Вычисляет суммарную тягу и момент по тягам двигателей.

    f = sum(motorThrusts(:));
    M = [quad.d * (motorThrusts(4) - motorThrusts(2)); ...
        quad.d * (motorThrusts(1) - motorThrusts(3)); ...
        quad.cTauF * (-motorThrusts(1) + motorThrusts(2) - motorThrusts(3) + motorThrusts(4))];
end
