%% Control Actions Rate of Change
clc, clear all, close all;

%% Load Matrices
separed = load("Separed_cost_without_velocities_0.5.mat");
dual = load("Dual_cost_without_velocities_0.5.mat");

% Control Values
F_dual = dual.F;
F_separed = separed.F;

M_dual = dual.M;
M_separed = separed.M;


% Time
t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(F_dual));
t_separed = t_separed(1, 1:length(F_separed));

%% Figures Colors Definition
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

% Aux variable experiments 
ts = t_dual(2) - t_dual(1);
% Number of experiments
number_experiments = size(F_dual, 1);

%% Computing time derivative of control action
Aux_control_actions_dual = zeros(30, 4, length(M_separed));
Aux_control_actions_separed = zeros(30, 4, length(M_separed));

for k=1:number_experiments
    Dual_F_data = reshape(F_dual(k, 1, :)/(9.8+20), 1, length(M_separed));
    Dual_Mx_data = reshape(M_dual(k, 1, :)/(0.1), 1, length(M_separed));
    Dual_My_data = reshape(M_dual(k, 2, :)/(0.1), 1, length(M_separed));
    Dual_Mz_data = reshape(M_dual(k, 3, :)/(0.1), 1, length(M_separed));
    
    Aux_control_actions_dual(k, 1, :) =  sqrt([0, diff(Dual_F_data)].^2) / 1;
    Aux_control_actions_dual(k, 2, :) =  sqrt([0, diff(Dual_Mx_data)].^2) / 1;
    Aux_control_actions_dual(k, 3, :) =  sqrt([0, diff(Dual_My_data)].^2) / 1;
    Aux_control_actions_dual(k, 4, :) =  sqrt([0, diff(Dual_Mz_data)].^2 )/ 1;
    
    Separed_F_data = reshape(F_separed(k, 1, :)/(9.8+20), 1, length(M_separed));
    Separed_Mx_data = reshape(M_separed(k, 1, :)/(0.1), 1, length(M_separed));
    Separed_My_data = reshape(M_separed(k, 2, :)/(0.1), 1, length(M_separed));
    Separed_Mz_data = reshape(M_separed(k, 3, :)/(0.1), 1, length(M_separed));
    
    Aux_control_actions_separed(k, 1, :) =  sqrt([0, diff(Separed_F_data)].^2) / 1;
    Aux_control_actions_separed(k, 2, :) =  sqrt([0, diff(Separed_Mx_data)].^2) / 1;
    Aux_control_actions_separed(k, 3, :) =  sqrt([0, diff(Separed_My_data)].^2) / 1;
    Aux_control_actions_separed(k, 4, :) =  sqrt([0, diff(Separed_Mz_data)].^2) / 1;
end


% Location Plots
dimension_x = [0.05, 0.34, 0.62, 0.89];
dimension_y = [1.0, 0.79,  0.56, 0.32];
%% plot Results
figure('Position', [500 500 sizeX sizeY]);
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;
colors = crameri('cork',number_experiments);

axes('Position',[dimension_x(1) dimension_y(1)-0.20  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_dual(k, 1, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("Rate of Change Control Actions DQ-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    ylabel('$||{f_z(k) - f_z(k-1)}||$','fontsize',12,'interpreter','latex', 'Color',C18);
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
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_separed(k, 1, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("Rate of Change Control Actions Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    %ylabel('$\frac{f_z(k) - f_z(k-1)}{t_s}$','fontsize',12,'interpreter','latex', 'Color',C18);
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

%% RMSE Orientation dual
Data_dual = reshape(Aux_control_actions_dual(1:number_experiments, 1, :), number_experiments, length(Aux_control_actions_separed));
Data_dual = Data_dual';


Data_separed = reshape(Aux_control_actions_separed(1:number_experiments, 1, :), number_experiments, length(Aux_control_actions_separed));
Data_separed = Data_separed';

% %% Computing ISE
axes('Position',[dimension_x(3) dimension_y(1)-0.20  .24 .15]);

[ISE_dual_plot, Ise_dual] = ISE(Data_dual, t_dual);
[ISE_separed_plot, Ise_separed] = ISE(Data_separed, t_dual);

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
title("ISE", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\frac{1}{N} \sum_{i=1}^{N}(\sum_{k=1}^{t}(({f_z(k) - f_z(k-1)})^2 ts))$','fontsize',12,'interpreter','latex', 'Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
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

axes('Position',[dimension_x(4) dimension_y(1)-0.20  .11 .16]);

rmseValues = [Ise_dual, Ise_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, length(Ise_dual)), repmat({'Baseline'}, 1, length(Ise_separed))];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
title("ISE", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\textrm{WRMSE Orientation}$','fontsize',12,'interpreter','latex', 'Color',C18);
%     xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
% Figure properties
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
axes('Position',[dimension_x(1) dimension_y(2)-0.16 .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_dual(k, 2, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    %title("Translation Error Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    ylabel('$||{\tau_x(k) - \tau_x(k-1)}||$','fontsize',12,'interpreter','latex', 'Color',C18);
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

axes('Position',[dimension_x(2) dimension_y(2)-0.16  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_separed(k, 2, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    %title("Translation Error Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    %ylabel('$\frac{\tau_x(k) - \tau_x(k-1)}{t_s}$','fontsize',14,'interpreter','latex', 'Color',C18);
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

%% RMSE Orientation dual
Data_dual = reshape(Aux_control_actions_dual(1:number_experiments, 2, :), number_experiments, length(Aux_control_actions_separed));
Data_dual = Data_dual';


Data_separed = reshape(Aux_control_actions_separed(1:number_experiments, 2, :), number_experiments, length(Aux_control_actions_separed));
Data_separed = Data_separed';

% %% Computing ISE
axes('Position',[dimension_x(3) dimension_y(2)-0.16  .24 .15]);

[ISE_dual_plot, Ise_dual] = ISE(Data_dual, t_dual);
[ISE_separed_plot, Ise_separed] = ISE(Data_separed, t_dual);

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
%title("ISE Rate of Change", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\frac{1}{N} \sum_{i=1}^{N}(\sum_{k=1}^{t}(({f_z(k) - f_z(k-1)})^2 ts))$','fontsize',12,'interpreter','latex', 'Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
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

axes('Position',[dimension_x(4) dimension_y(2)-0.15  .11 .14]);

rmseValues = [Ise_dual, Ise_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, length(Ise_dual)), repmat({'Baseline'}, 1, length(Ise_separed))];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
%title("ISE", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\textrm{WRMSE Orientation}$','fontsize',12,'interpreter','latex', 'Color',C18);
%     xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
% Figure properties
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


axes('Position',[dimension_x(1) dimension_y(3)-0.10  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_dual(k, 3, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    %title("Translation Error Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    ylabel('$||{\tau_y(k) - \tau_y(k-1)}||$','fontsize',12,'interpreter','latex', 'Color',C18);
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

axes('Position',[dimension_x(2) dimension_y(3)-0.10  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_separed(k, 3, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    %title("Translation Error Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    %ylabel('$\frac{\tau_x(k) - \tau_x(k-1)}{t_s}$','fontsize',14,'interpreter','latex', 'Color',C18);
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

%% RMSE Orientation dual
Data_dual = reshape(Aux_control_actions_dual(1:number_experiments, 3, :), number_experiments, length(Aux_control_actions_separed));
Data_dual = Data_dual';


Data_separed = reshape(Aux_control_actions_separed(1:number_experiments, 3, :), number_experiments, length(Aux_control_actions_separed));
Data_separed = Data_separed';

% %% Computing ISE
axes('Position',[dimension_x(3) dimension_y(3)-0.10  .24 .15]);

[ISE_dual_plot, Ise_dual] = ISE(Data_dual, t_dual);
[ISE_separed_plot, Ise_separed] = ISE(Data_separed, t_dual);

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
%title("ISE Rate of Change", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\frac{1}{N} \sum_{i=1}^{N}(\sum_{k=1}^{t}(({f_z(k) - f_z(k-1)})^2 ts))$','fontsize',12,'interpreter','latex', 'Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
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

axes('Position',[dimension_x(4) dimension_y(3)-0.09  .11 .14]);

rmseValues = [Ise_dual, Ise_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, length(Ise_dual)), repmat({'Baseline'}, 1, length(Ise_separed))];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
%title("ISE", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\textrm{WRMSE Orientation}$','fontsize',12,'interpreter','latex', 'Color',C18);
%     xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
% Figure properties
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


axes('Position',[dimension_x(1) dimension_y(4)-0.03  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_dual(k, 4, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    %title("Translation Error Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    ylabel('$||{\tau_z(k) - \tau_z(k-1)}||$','fontsize',12,'interpreter','latex', 'Color',C18);
    xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
    %% Figure properties
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
    
end

axes('Position',[dimension_x(2) dimension_y(4)-0.03  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_control_actions_separed(k, 4, :), 1, length(Aux_control_actions_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    %title("Translation Error Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
    %ylabel('$\frac{\tau_x(k) - \tau_x(k-1)}{t_s}$','fontsize',14,'interpreter','latex', 'Color',C18);
    xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
    %% Figure properties
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
    
end

%% RMSE Orientation dual
Data_dual = reshape(Aux_control_actions_dual(1:number_experiments, 4, :), number_experiments, length(Aux_control_actions_separed));
Data_dual = Data_dual';


Data_separed = reshape(Aux_control_actions_separed(1:number_experiments, 4, :), number_experiments, length(Aux_control_actions_separed));
Data_separed = Data_separed';

% %% Computing ISE
axes('Position',[dimension_x(3) dimension_y(4)-0.03  .24 .15]);

[ISE_dual_plot, Ise_dual] = ISE(Data_dual, t_dual);
[ISE_separed_plot, Ise_separed] = ISE(Data_separed, t_dual);

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
%title("ISE Rate of Change", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
%ylabel('$\frac{1}{N} \sum_{i=1}^{N}(\sum_{k=1}^{t}(({f_z(k) - f_z(k-1)})^2 ts))$','fontsize',12,'interpreter','latex', 'Color',C18);
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

axes('Position',[dimension_x(4) dimension_y(4)-0.04  .11 .15]);

rmseValues = [Ise_dual, Ise_separed]; % Concatenate RMSE values
% Create grouping variable
groups = [repmat({'DQ'}, 1, length(Ise_dual)), repmat({'Baseline'}, 1, length(Ise_separed))];
boxplot(rmseValues, groups); % Create the boxplot
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.3*fontsizeTicks)
%title("ISE", 'fontsize', 14, 'interpreter', 'latex', 'Color', 'black');
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
set(gcf, 'Color', 'w'); % Sets axes background
export_fig Rate_of_change_0.5N.pdf -q101