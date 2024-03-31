clc, clear all, close 
% Sample data generation for demonstration
% Assuming each column is an observation for a different experiment
numObservations = 100; % Number of observations (e.g., time points)
numExperiments = 10; % Number of experiments or trials

% Simulate signals and references with noise
signal1 = randn(numObservations, numExperiments);
reference1 = signal1 + randn(numObservations, numExperiments) * 0.1; % Reference signal with some noise

signal2 = randn(numObservations, numExperiments);
reference2 = signal2 + randn(numObservations, numExperiments) * 0.1; % Reference signal with some noise

% Calculate RMSE for each observation (column-wise) for both signals
rmse1 = sqrt(mean((signal1 - reference1).^2));
rmse2 = sqrt(mean((signal2 - reference2).^2));

% Combine RMSE values
rmseValues = [rmse1, rmse2]; % Concatenate RMSE values

% Create grouping variable
groups = [repmat({'Signal 1'}, 1, numExperiments), repmat({'Signal 2'}, 1, numExperiments)];

figure; % Create a new figure
boxplot(rmseValues, groups); % Create the boxplot
title('RMSE Comparison Between Two Signals');
ylabel('RMSE');
xlabel('Signal');