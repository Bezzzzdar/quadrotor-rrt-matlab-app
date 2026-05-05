function ctrl = defaultController()
%DEFAULTCONTROLLER Возвращает коэффициенты регулятора слежения.

ctrl.kpPos = [0.8; 0.8; 1.5];
ctrl.kdPos = [2; 2; 2.6];
ctrl.kR = 8.81;
ctrl.kOmega = 2.54;
end
