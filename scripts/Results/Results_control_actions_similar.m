%% Results DQ and Classical Formulation
clc, clear all, close all;

%% Load Matrices
separed = load("Separed_cost_without_velocities_0.5.mat");
dual = load("Dual_cost_without_velocities_0.5.mat");

% Values
F_dual = dual.F;
F_separed = separed.F;

M_dual = dual.M;
M_separed = separed.M;


t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(F_dual));
t_separed = t_separed(1, 1:length(F_separed));


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
% % Aux variable experiments 
experiments = [29, 10, 13;...
               20, 18, 28];
% Aux variable experiments 
% experiments = [1, 2, 6;...
%                11, 14, 15];
dimension_x = [0.05, 0.38, 0.71];
dimension_y = [1.0, 0.5];
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;
    for j=1:length(dimension_y)
        for k=1:length(dimension_x)
        axes('Position',[dimension_x(k) dimension_y(j)-0.13  .25 .10]);
        %% Data generation
        F_separed_plot = line(t_separed,reshape(F_separed(experiments(j, k), 1, :), 1, length(F_separed)));
        set(F_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        F_dual_plot = line(t_dual,reshape(F_dual(experiments(j, k), 1, :), 1, length(F_dual)));
        set(F_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        ylabel('${[N]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);

        %% Legend nomeclature
        hLegend_1 = legend([F_separed_plot,F_dual_plot],{'$f_{z-classic}$','$f_{z-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
         set(gca,'ticklabelinterpreter','latex',...
                 'fontsize',1.3*fontsizeTicks)
        aux_title = string(experiments(j, k));
        title('Experiment ' + aux_title, 'fontsize', 12, 'interpreter', 'latex', 'Color', 'black');
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

        axes('Position',[dimension_x(k) dimension_y(j)-0.23 .25 .08]);
        %% Data generation
        M_separed_plot = line(t_separed,reshape(M_separed(experiments(j, k), 1, :), 1, length(M_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(M_dual(experiments(j, k), 1, :), 1, length(M_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        ylabel('${[N.m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);

        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$\tau_{x-classic}$','$\tau_{x-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
         set(gca,'ticklabelinterpreter','latex',...
                 'fontsize',1.3*fontsizeTicks)
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

        axes('Position',[dimension_x(k)  dimension_y(j)-0.33 .25 .08]);
        %% Data generation
        M_separed_plot = line(t_separed,reshape(M_separed(experiments(j, k), 2, :), 1, length(M_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(M_dual(experiments(j, k), 2, :), 1, length(M_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        ylabel('${[N.m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);

        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$\tau_{y-classic}$','$\tau_{y-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
         set(gca,'ticklabelinterpreter','latex',...
                 'fontsize',1.3*fontsizeTicks)
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

        axes('Position',[dimension_x(k)  dimension_y(j)-0.43 .25 .08]);
        %% Data generation
        M_separed_plot = line(t_separed,reshape(M_separed(experiments(j, k), 3, :), 1, length(M_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(M_dual(experiments(j, k), 3, :), 1, length(M_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        ylabel('${[N.m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);

        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$\tau_{z-classic}$','$\tau_{z-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
         set(gca,'ticklabelinterpreter','latex',...
                 'fontsize',1.3*fontsizeTicks)
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
    end

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Control_actions_comparative_similar_behavior_0.5N.pdf -q101
