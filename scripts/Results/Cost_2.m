%% Cost of the experiments Dualquaternions and Separed cost Function

clc, clear all, close all;

%% Load Matrices
separed = load("Separed_cost_without_velocities_0.5.mat");
dual = load("Dual_cost_without_velocities_0.5.mat");

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
number_experiments = size(Aux_cost_dual, 1);

% Location Plots
dimension_x = [0.05, 0.33, 0.71];
dimension_y = [1.0, 0.8,  0.58, 0.35];
%% plot Results
figure('Position', [500 500 sizeX sizeY]);
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;
colors = crameri('cork',number_experiments);


%% RMSE Translation dual
Data_dual = reshape(Aux_cost_dual(1:number_experiments, 1, :), number_experiments, length(Aux_cost_dual));
Data_dual = Data_dual';

Data_separed = reshape(Aux_cost_separed(1:number_experiments, 1, :), number_experiments, length(Aux_cost_separed));
Data_separed = Data_separed';

axes('Position',[dimension_x(1) dimension_y(3)-0.20  .25 .18]);

% Computing ITSE
[ISE_dual_plot, Ise_dual] = ISE_t(Data_dual, t_dual);
[ISE_separed_plot, Ise_separed] = ISE_t(Data_separed, t_dual);

x_vector_dual = [t_dual, fliplr(t_dual)];
patch_dual = fill(x_vector_dual, [ISE_dual_plot,fliplr(ISE_dual_plot*0)], colors(1, :));

set(patch_dual, 'FaceAlpha', 0.2);
hold on

x_vector_separed = [t_dual, fliplr(t_dual)];
patch_separed = fill(x_vector_separed, [ISE_separed_plot,fliplr(ISE_dual_plot*0)], colors(10, :));

set(patch_separed, 'FaceAlpha', 0.2);


F_dual_plot = line(t_dual, ISE_dual_plot);
set(F_dual_plot, 'LineStyle', '-', 'Color', colors(1,:), 'LineWidth', 1*lw);
F_separed_plot = line(t_dual, ISE_separed_plot);
set(F_separed_plot, 'LineStyle', '-', 'Color', colors(10,:), 'LineWidth', 1*lw);
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
title("ITSE Translation Error", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
hLegend_1 = legend([F_dual_plot, F_separed_plot,],{'$DQ$','$Baseline$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black', 'Location', 'best');
ylabel('$\frac{1}{N} \sum_{i=1}^{N}(\sum_{k=1}^{t}(t(k)^{2} ||~\mathbf{t}^{i}_{d, k}- \textrm{trans}(\mathbf{x}^{i}_k)|| t_s))$','fontsize',12,'interpreter','latex', 'Color',C18);
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

axes('Position',[dimension_x(1)+0.08 dimension_y(4)-0.25  .12 .18]);

% Combine WRMSE values
Data_dual_weight = t_dual'.*Data_dual;
Data_separed_weight = t_dual'.*Data_separed;

rmse_dual = sqrt(sum(Data_dual_weight)/sum(t_dual));
rmse_separed = sqrt(sum(Data_separed_weight)/sum(t_dual));

rmseValues = [rmse_dual, rmse_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, length(Ise_dual)), repmat({'Baseline'}, 1, length(Ise_separed))];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
title("W-RMSE Translation", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\textrm{WRMSE Translation}$','fontsize',12,'interpreter','latex', 'Color',C18);
%     xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);

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

%% RMSE Orientation dual
Data_dual = reshape(Aux_cost_dual(1:number_experiments, 2, :), number_experiments, length(Aux_cost_dual));
Data_dual = Data_dual';


Data_separed = reshape(Aux_cost_separed(1:number_experiments, 2, :), number_experiments, length(Aux_cost_separed));
Data_separed = Data_separed';

% %% Computing ISE
axes('Position',[dimension_x(2)+0.05 dimension_y(3)-0.20  .25 .18]);

[ISE_dual_plot, Ise_dual] = ISE_t(Data_dual, t_dual);
[ISE_separed_plot, Ise_separed] = ISE_t(Data_separed, t_dual);

x_vector_dual = [t_dual, fliplr(t_dual)];
patch_dual = fill(x_vector_dual, [ISE_dual_plot,fliplr(ISE_dual_plot*0)], colors(1, :));

set(patch_dual, 'FaceAlpha', 0.2);
hold on

x_vector_separed = [t_dual, fliplr(t_dual)];
patch_separed = fill(x_vector_separed, [ISE_separed_plot,fliplr(ISE_dual_plot*0)], colors(10, :));

set(patch_separed, 'FaceAlpha', 0.2);


F_dual_plot = line(t_dual, ISE_dual_plot);
set(F_dual_plot, 'LineStyle', '-', 'Color', colors(1,:), 'LineWidth', 1*lw);
F_separed_plot = line(t_dual, ISE_separed_plot);
set(F_separed_plot, 'LineStyle', '-', 'Color', colors(10,:), 'LineWidth', 1*lw);



set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
hLegend_1 = legend([F_dual_plot, F_separed_plot,],{'$DQ$','$Baseline$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black', 'Location', 'best');
title("ITSE Orientation Error", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
ylabel('$\frac{1}{N} \sum_{i=1}^{N}(\sum_{k=1}^{t}(t(k)^{2} ||\textrm{Log}(\mathbf{q}^{i*}_{d, k} \circ \textrm{quat}(\mathbf{x}^{i}_k))|| t_s))$','fontsize',12,'interpreter','latex', 'Color',C18);
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

axes('Position',[dimension_x(2)+0.1 dimension_y(4)-0.25  .12 .18]);


% Combine WRMSE values
Data_dual_weight = t_dual'.*Data_dual;
Data_separed_weight = t_dual'.*Data_separed;

rmse_dual = sqrt(sum(Data_dual_weight)/sum(t_dual));
rmse_separed = sqrt(sum(Data_separed_weight)/sum(t_dual));

rmseValues = [rmse_dual, rmse_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, length(Ise_dual)), repmat({'Baseline'}, 1, length(Ise_separed))];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
title("W-RMSE Orientation", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\textrm{WRMSE Orientation}$','fontsize',12,'interpreter','latex', 'Color',C18);
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
% % 
set(gcf, 'Color', 'w'); % Sets axes background
export_fig ISE_0.5N.pdf -q101