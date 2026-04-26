function ctrl = defaultController()
%DEFAULTCONTROLLER Возвращает коэффициенты регулятора слежения.

ctrl.kpPos = [0.8; 0.8; 1.5];
ctrl.kdPos = [1.8; 1.8; 2.4];
ctrl.kR = 8.81;
ctrl.kOmega = 2.54;
end
