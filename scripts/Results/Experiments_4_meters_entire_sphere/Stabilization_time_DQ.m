%% COmputing stabilization time of the translation and orientation Error
clc, clear all, close all;


%% Load Matrices
separed = load("Separed_cost.mat");
dual = load("Dual_cost.mat");

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


%% Figures Colors Definition
lw = 1; % linewidth 1
lwV = 2; % linewidth 2zz
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; % size figure
sizeY = 1000; % size figure

% color propreties
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;

C18 = [0 0 0];
c5 = [130, 37, 37]/255;
c6 = [205, 167, 37]/255;
c7 = [81, 115, 180]/255;

C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;
% 

% Number of experiments
number_experiments = size(cost_t_dual, 1);

% Location Plots
dimension_x = [0.05, 0.33, 0.71];
dimension_y = [1.0, 0.8,  0.58, 0.35];
%% plot Results
% figure('Position', [500 500 sizeX sizeY])
% set(gcf, 'Position', [500 500 sizeX sizeY]);
% fig1_comps.fig = gcf;
% 
% 
% 
% axes('Position',[dimension_x(1) dimension_y(1)-0.20  .25 .15]);
% colors = crameri('cork',number_experiments);

%% Computing stabilization time
stabilization_time_dual = zeros(number_experiments, 2);
stabilization_time_separed = zeros(number_experiments, 2);
percentage = 10; % Stabilization within 2% of the final value
finalSamples = 250; % Use the last 5 samples to calculate the final value


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



% % Combine RMSE values
%% plot Results
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[dimension_x(1)+0.05 dimension_y(3)-0.25  .12 .18]);

rmse_dual = ((stabilization_time_dual(:, 1)));
rmse_separed = ((stabilization_time_separed(:, 1)));
% 
rmseValues = [rmse_dual, rmse_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, number_experiments), repmat({'Baseline'}, 1, number_experiments)];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
title("Settling Time Trasnlation", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
ylabel('$\textrm{[seg]}$','fontsize',12,'interpreter','latex', 'Color',C18);
%     xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;

axes('Position',[dimension_x(2) dimension_y(3)-0.25  .12 .18]);


% % Combine RMSE values

rmse_dual = ((stabilization_time_dual(:, 2)));
rmse_separed = ((stabilization_time_separed(:, 2)));
% 
rmseValues = [rmse_dual, rmse_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, number_experiments), repmat({'Baseline'}, 1, number_experiments)];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
title(" Settling Time Orientation", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\textrm{[s]}$','fontsize',12,'interpreter','latex', 'Color',C18);
%     xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Stabilization_time_0.5N.pdf -q101