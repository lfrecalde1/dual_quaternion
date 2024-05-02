%% KKT conditions of the experiments Dualquaternions and Separed cost Function
clc, clear all, close all;

%% Load Matrices
separed = load("Separed_cost.mat");
dual = load("Dual_cost.mat");

% Cost over time position
KKT_dual = dual.KKT;
KKT_separed = separed.KKT;


% Cost Values position quaternion and 
Aux_KKT_dual = zeros(size(KKT_dual, 1), 4, length(KKT_dual));
Aux_KKT_separed = zeros(size(KKT_separed, 1), 4, length(KKT_dual));

for i=1:size(KKT_dual, 1)
    for j=1:size(KKT_dual, 3)
        Aux_KKT_dual(i, 1, j) = log(KKT_dual(i, 1, j));
        Aux_KKT_separed(i, 1, j) = log(KKT_separed(i, 1, j));
        
        Aux_KKT_dual(i, 2, j) = log(KKT_dual(i, 2, j));
        Aux_KKT_separed(i, 2, j) = log(KKT_separed(i, 2, j));
        
        Aux_KKT_dual(i, 3, j) = log(KKT_dual(i, 3, j));
        Aux_KKT_separed(i, 3, j) = log(KKT_separed(i, 3, j));
        
        Aux_KKT_dual(i, 4, j) = KKT_dual(i, 4, j);
        Aux_KKT_separed(i, 4, j) = KKT_separed(i, 4, j);
    end
    
end

t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(Aux_KKT_dual));
t_separed = t_separed(1, 1:length(Aux_KKT_separed));


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
number_experiments = size(Aux_KKT_dual, 1);

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
    F_dual_plot = line(t_dual,reshape(Aux_KKT_dual(k, 1, :), 1, length(Aux_KKT_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("Stationarity Residual DQ-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
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
    F_dual_plot = line(t_dual,reshape(Aux_KKT_separed(k, 1, :), 1, length(Aux_KKT_separed)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("Stationarity Residual Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
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

axes('Position',[dimension_x(1) dimension_y(2)-0.20  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_KKT_dual(k, 2, :), 1, length(Aux_KKT_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("Equality Constraints Residual DQ-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
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

axes('Position',[dimension_x(2) dimension_y(2)-0.20  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_KKT_separed(k, 2, :), 1, length(Aux_KKT_separed)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    title("Equality Constraints Residual Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
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

axes('Position',[dimension_x(1) dimension_y(3)-0.20  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_KKT_dual(k, 3, :), 1, length(Aux_KKT_dual)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
    title("Inequality Constraints Residual DQ-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
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

axes('Position',[dimension_x(2) dimension_y(3)-0.20  .25 .15]);
for k=1:number_experiments
    
    %% Data generation
    F_dual_plot = line(t_dual,reshape(Aux_KKT_separed(k, 3, :), 1, length(Aux_KKT_separed)));
    set(F_dual_plot, 'LineStyle', '-', 'Color', colors(k,:), 'LineWidth', 1*lw);
    %% Legend nomeclature
    set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
    xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
    title("Inequality Constraints Residual Baseline-NMPC", 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
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


set(gcf, 'Color', 'w'); % Sets axes background
export_fig KKT.pdf -q101