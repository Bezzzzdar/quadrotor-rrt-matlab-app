%RUN_APP Запуск MATLAB App.

clear;
clc;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, 'src')));
addpath(fullfile(rootDir, 'apps'));

app = QuadrotorRRTApp(); %#ok<NASGU>
