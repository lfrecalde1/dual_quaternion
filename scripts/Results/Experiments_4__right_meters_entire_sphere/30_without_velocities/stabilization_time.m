function [stabilization_time_separed, stabilization_time_dual] = stabilization_time(separed, dual)
%UNTITLED2 Summary of this function goes here

% Cost over time position
cost_t_dual = dual.translation_cost;
cost_t_separed = separed.translation_cost;

% Cost over time quaternion
cost_q_dual = dual.orientation_cost;
cost_q_separed = separed.orientation_cost;

% Cost Values position quaternion and 
Aux_cost_dual = zeros(30, 3, length(cost_t_dual));
Aux_cost_separed = zeros(30, 3, length(cost_t_dual));

for i=1:size(cost_t_dual, 1)
    for j=1:size(cost_t_dual, 3)
        Aux_cost_dual(i, 1, j) = cost_t_dual(i, 1, j);
        Aux_cost_dual(i, 2, j) = cost_q_dual(i, 1, j);
        Aux_cost_dual(i, 3, j) = cost_t_dual(i, 1, j) +  cost_q_dual(i, 1, j);
        
        Aux_cost_separed(i, 1, j) = cost_t_separed(i, 1, j);
        Aux_cost_separed(i, 2, j) = cost_q_separed(i, 1, j);
        Aux_cost_separed(i, 3, j) = cost_t_separed(i, 1, j) +  cost_q_separed(i, 1, j);
    end
    
end
t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(Aux_cost_dual));
t_separed = t_separed(1, 1:length(Aux_cost_separed));


% Number of experiments
number_experiments = size(cost_t_dual, 1);

%% Computing stabilization time
stabilization_time_dual = zeros(number_experiments, 2);
stabilization_time_separed = zeros(number_experiments, 2);

for k=1:number_experiments
    signal_data_dual_position = reshape(Aux_cost_dual(k, 1, :), 1, length(Aux_cost_dual));
    signal_data_dual_orientation = reshape(Aux_cost_dual(k, 2, :), 1, length(Aux_cost_dual));

    signal_data_separed_position = reshape(Aux_cost_separed(k, 1, :), 1, length(Aux_cost_dual));
    signal_data_separed_orientation = reshape(Aux_cost_separed(k, 2, :), 1, length(Aux_cost_dual));

    stabilization_time_dual(k, 1) = calculateStabilizationTime(signal_data_dual_position, t_dual, 0.08, -0.08);
    stabilization_time_dual(k, 2) = calculateStabilizationTime(signal_data_dual_orientation, t_dual, 0.005, -0.005);

    stabilization_time_separed(k, 1) = calculateStabilizationTime(signal_data_separed_position, t_dual, 0.08, -0.08);
    stabilization_time_separed(k, 2) = calculateStabilizationTime(signal_data_separed_orientation, t_dual, 0.005, -0.005);
end



end

