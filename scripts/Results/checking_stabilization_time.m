%% Code to compute stabilization time of my signals
clc, clear all, close all;

% Create signal
signal = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 100, 100, 100]; % Example signal vector
time = 0:length(signal)-1; % Corresponding time vector
percentage = 2; % Stabilization within 2% of the final value
finalSamples = 3; % Use the last 5 samples to calculate the final value

stabilizationTime = calculateStabilizationTime(signal, time, percentage, finalSamples);
% 
% fprintf('Stabilization Time: %f seconds\n', stabilizationTime);