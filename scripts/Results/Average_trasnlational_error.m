clc, clear all, close all;
% Example Data: Columns represent [x, y, z] coordinates
% Ground truth positions
true_positions = [
    1, 2, 3;
    2, 3, 4;
    3, 4, 5;
    % Add as many as you have
];

% Estimated positions
estimated_positions = [
    1., 2., 3.;
    2.0, 3, 4.1;
    3, 4, 5.0;
    % Corresponding estimates
];

% Calculate the Euclidean distances (errors) between true and estimated positions
errors = sqrt(sum((true_positions - estimated_positions).^2, 2));

% Compute the Average Translational Error (ATE)
ATE = mean(errors);

disp(['Average Translational Error (ATE): ', num2str(ATE)]);