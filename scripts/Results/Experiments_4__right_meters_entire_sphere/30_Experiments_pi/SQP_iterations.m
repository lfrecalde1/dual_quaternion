%% SQP iterations of the experiments Dualquaternions and Separed cost Function
clc, clear all, close all;

%% Load Matrices
separed = load("Separed_cost.mat");
dual = load("Dual_cost.mat");

% Cost over time position
sqp_iter_dual = dual.sqp;
sqp_iter_separed = separed.sqp;


% Cost Values position quaternion and 
Aux_sqp_dual = zeros(size(sqp_iter_dual, 1), 1, length(sqp_iter_dual));
Aux_sqp_separed = zeros(size(sqp_iter_separed, 1), 1, length(sqp_iter_dual));

for i=1:size(sqp_iter_dual, 1)
    for j=1:size(sqp_iter_dual, 3)
        Aux_sqp_dual(i, 1, j) = sqp_iter_dual(i, 1, j);
        Aux_sqp_separed(i, 1, j) = sqp_iter_separed(i, 1, j);
    end
    
end

t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(Aux_sqp_dual));
t_separed = t_separed(1, 1:length(Aux_sqp_separed));


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
number_experiments = size(Aux_sqp_dual, 1);

% Location Plots
dimension_x = [0.05, 0.36, 0.71];
dimension_y = [1.0, 0.8,  0.58, 0.35];
%% plot Results
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;



axes('Position',[dimension_x(1) dimension_y(1)-0.20  .25 .15]);
colors = crameri('hawaii',number_experiments);

for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_sqp_dual(k, 1, :), 1, length(Aux_sqp_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("SQP Iterations DQ-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    %% Figure properties
    ax_1 = gca;
    ax_1.Box = 'on';
    ax_1.BoxStyle = 'full';
    ax_1.TickLength = [0.01;0.01];
    ax_1.XTickLabel = [];
    ax_1.TickDirMode = 'auto';
    ax_1.YMinorTick = 'on';
    ax_1.XMinorTick = 'on';
    ax_1.XMinorGrid = 'on';
    ax_1.YMinorGrid = 'on';
    ax_1.MinorGridAlpha = 0.15;
    ax_1.LineWidth = 0.8;
    ax_1.XLim = [t_dual(1), t_dual(end)]; % Set limits for x-axis
    
end

axes('Position',[dimension_x(2) dimension_y(1)-0.20  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_sqp_separed(k, 1, :), 1, length(Aux_sqp_separed)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("SQP Iterations Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    %ylabel('$||~\mathbf{t}_{d, k}- \textrm{trans}(\mathbf{x}_k)||^{2}$','fontsize',12,'interpreter','latex', 'Color',C18);
    %xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
    %% Figure properties
    ax_1 = gca;
    ax_1.Box = 'on';
    ax_1.BoxStyle = 'full';
    ax_1.XTickLabel = [];
    ax_1.TickLength = [0.01;0.01];
    ax_1.TickDirMode = 'auto';
    ax_1.YMinorTick = 'on';
    ax_1.XMinorTick = 'on';
    ax_1.XMinorGrid = 'on';
    ax_1.YMinorGrid = 'on';
    ax_1.MinorGridAlpha = 0.15;
    ax_1.LineWidth = 0.8;
    ax_1.XLim = [t_dual(1), t_dual(end)]; % Set limits for x-axis
    
end

% end
%% RMSE Translation dual
Data_dual = reshape(Aux_sqp_dual(:, 1, :), number_experiments, length(Aux_sqp_dual));
Data_dual = Data_dual';
% %% Computing RMS
axes('Position',[dimension_x(1) dimension_y(2)-0.22  .25 .18]);
data_mean_dual = mean(Data_dual');
rmse_dual = (data_mean_dual);
std_dual = std(Data_dual');

% Plooting STD
x_vector_dual = [t_dual, fliplr(t_dual)];
patch_dual = fill(x_vector_dual, [rmse_dual+std_dual,fliplr(rmse_dual-std_dual)], colors(1, :));
% patch_separed = fill(x_vector_separed, [rmse_separed+std_separed,fliplr(rmse_separed-std_separed)], colors(2, :));

F_dual_plot = line(t_dual, rmse_dual);
set(F_dual_plot, 'LineStyle', '-', 'Color', colors(1,:), 'LineWidth', 1*lw);

set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
set(patch_dual, 'edgecolor', 'none');
set(patch_dual, 'FaceAlpha', 0.2);
title("MSE-STD SQP Iterations DQ-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
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
ax_1.XLim = [t_dual(1), t_dual(end)]; % Set limits for x-axis

%% RMSE Separed Translation
Data_separed = reshape(Aux_sqp_separed(:, 1, :), number_experiments, length(Aux_sqp_separed));
Data_separed = Data_separed';


%% Computing RMS
axes('Position',[dimension_x(2) dimension_y(2)-0.22  .25 .18]);
data_mean_separed = mean(Data_separed');
rmse_separed = (data_mean_separed);
std_separed = std(Data_separed');

% Plooting STD
x_vector_separed = [t_dual, fliplr(t_dual)];
patch_separed = fill(x_vector_separed, [rmse_separed+std_separed,fliplr(rmse_separed-std_separed)], colors(7, :));
% patch_separed = fill(x_vector_separed, [rmse_separed+std_separed,fliplr(rmse_separed-std_separed)], colors(2, :));

F_dual_plot = line(t_dual, rmse_separed);
set(F_dual_plot, 'LineStyle', '-', 'Color', colors(7,:), 'LineWidth', 1*lw);

set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
set(patch_separed, 'edgecolor', 'none');
set(patch_separed, 'FaceAlpha', 0.2);
title("MSE-STD SQP Iterations Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
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
ax_1.XLim = [t_dual(1), t_dual(end)]; % Set limits for x-axis


% 
set(gcf, 'Color', 'w'); % Sets axes background
export_fig SQP_iter.pdf -q101
