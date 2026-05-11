%RUN_ANALYSIS Сравнение параметров алгоритма RRT

clear;
clc;
close all;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));

scenario = config.defaultScenario();

