% Log behavior 
clc, clear all, close all;

% Example vector with values spanning several orders of magnitude
vector = [0.1, 10, 1000, 100000];

% Applying the natural logarithm to standardize magnitudes
log_vector = log(vector);

figure
plot(vector')

figure
plot(log_vector')

disp('Original Vector:');
disp(vector);
disp('Log-Transformed Vector:');
disp(log_vector);