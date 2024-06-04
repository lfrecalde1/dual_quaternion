function save_all_results(experiments)

%% Load Matrices
separed = load("Separed_cost.mat");
dual = load("Dual_cost.mat");

% Control Values
F_dual = dual.F;
F_separed = separed.F;

M_dual = dual.M;
M_separed = separed.M;


t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(F_dual));
t_separed = t_separed(1, 1:length(F_separed));


% States of the system
X_dual = dual.x;
X_separed = separed.x;

Aux_X_dual = zeros(30, 7, length(X_dual));
Aux_X_separed = zeros(30, 7, length(X_separed));

for i=1:size(Aux_X_dual, 1)
    for j=1:size(Aux_X_dual, 3)
        position = X_separed(i, 1:3, j);
        quaternion = X_separed(i, 7:10, j);
        Aux_X_separed(i, :, j) = [position, quaternion]';
    end
    
end

for i=1:size(Aux_X_dual, 1)
    for j=1:size(Aux_X_dual, 3)
        quaternion = X_dual(i, 1:4, j)';
        position = get_traslatation_dual(X_dual(i, 1:8, j));
        Aux_X_dual(i, :, j) = [position(2:4); quaternion];
    end
    
end
cost_t_dual = dual.translation_cost;
cost_t_separed = separed.translation_cost;

cost_q_dual = dual.orientation_cost;
cost_q_separed = separed.orientation_cost;

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
%% Figures Colors Definition
lw = 1; % linewidth 1
lwV = 2; % linewidth 2zz
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; % size figure
sizeY = 750; % size figure

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
% experiments = [29, 10, 13;...
%                20, 18, 28];
% Aux variable experiments

dimension_x = [0.05, 0.38, 0.71];
dimension_y = [1.0, 0.5];
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;
colors = crameri('cork',size(F_separed, 1));
names = experiments(1,1);
for j=1:length(dimension_y)
    for k=1:length(dimension_x)
        axes('Position',[dimension_x(k) dimension_y(j)-0.13  .25 .10]);
        %% Data generation
        F_separed_plot = line(t_separed,reshape(F_separed(experiments(j, k), 1, :)/(9.8+20), 1, length(F_separed)));
        set(F_separed_plot, 'LineStyle', '-', 'Color', colors(1, :), 'LineWidth', 1.2*lw);
        F_dual_plot = line(t_dual,reshape(F_dual(experiments(j, k), 1, :)/(9.8+20), 1, length(F_dual)));
        set(F_dual_plot, 'LineStyle', '-', 'Color', colors(20, :), 'LineWidth', 1.2*lw);
        
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([F_separed_plot,F_dual_plot],{'$f_{z-baseline}$','$f_{z-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        ylabel('${[N]}$','fontsize',10,'interpreter','latex', 'Color',C18);
        aux_title = string(experiments(j, k));
        initial_conditions_x = string(round(Aux_X_separed(experiments(j, k), 1, 1), 1));
        initial_conditions_y = string(round(Aux_X_separed(experiments(j, k), 2, 1), 1));
        initial_conditions_z = string(round(Aux_X_separed(experiments(j, k), 3, 1), 1));
        
        initial_conditions_qw = string(round(Aux_X_separed(experiments(j, k), 4, 1), 1));
        initial_conditions_qx = string(round(Aux_X_separed(experiments(j, k), 5, 1), 1));
        initial_conditions_qy = string(round(Aux_X_separed(experiments(j, k), 6, 1), 1));
        initial_conditions_qz = string(round(Aux_X_separed(experiments(j, k), 7, 1), 1));
        title(['Experiment '+ aux_title + "~$\mathbf{p}_0 = [$" + initial_conditions_x + ", " + initial_conditions_y + ", " + initial_conditions_z + "$]$" + "~$\mathbf{q}_0 = [$" + initial_conditions_qw + ", " + initial_conditions_qx + ", " + initial_conditions_qy + ", " + initial_conditions_qz + "$]$"], 'FontSize', 11, 'Interpreter', 'latex', 'Color', 'black');
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
        M_separed_plot = line(t_separed,reshape(M_separed(experiments(j, k), 1, :)/0.1, 1, length(M_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', colors(1, :), 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(M_dual(experiments(j, k), 1, :)/0.1, 1, length(M_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', colors(20, :), 'LineWidth', 1.2*lw);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$\tau_{x-baseline}$','$\tau_{x-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        ylabel('${[N.m]}$','fontsize',10,'interpreter','latex', 'Color',C18);
        
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
        M_separed_plot = line(t_separed,reshape(M_separed(experiments(j, k), 2, :)/0.1, 1, length(M_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', colors(1, :), 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(M_dual(experiments(j, k), 2, :)/0.1, 1, length(M_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', colors(20, :), 'LineWidth', 1.2*lw);
        ylabel('${[N.m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$\tau_{y-baseline}$','$\tau_{y-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        ylabel('${[N.m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        
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
        M_separed_plot = line(t_separed,reshape(M_separed(experiments(j, k), 3, :)/0.1, 1, length(M_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', colors(1, :), 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(M_dual(experiments(j, k), 3, :)/0.1, 1, length(M_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', colors(20, :), 'LineWidth', 1.2*lw);
        ylabel('${[N.m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        xlabel('$\textrm{Time}[s]$','fontsize',8,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$\tau_{z-baseline}$','$\tau_{z-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        ylabel('${[N.m]}$','fontsize',11,'interpreter','latex', 'Color',C18);
        xlabel('$\textrm{Time}[s]$','fontsize',11,'interpreter','latex','Color',C18);
        
        %% Figure properties
        ax_1 = gca;
        ax_1.Box = 'on';
        ax_1.BoxStyle = 'full';
        ax_1.TickLength = [0.01;0.01];
        %ax_1.XTickLabel = [];
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
name_control = "Control_actions_comparative_" + string(names) + ".pdf";
export_fig(name_control, '-pdf');


%% Positions
t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(Aux_X_dual));
t_separed = t_separed(1, 1:length(Aux_X_separed));

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;
for j=1:length(dimension_y)
    for k=1:length(dimension_x)
        axes('Position',[dimension_x(k) dimension_y(j)-0.13  .25 .10]);
        %% Data generation
        F_separed_plot = line(t_separed,reshape(Aux_X_separed(experiments(j, k), 1, :), 1, length(Aux_X_separed)));
        set(F_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        F_dual_plot = line(t_dual,reshape(Aux_X_dual(experiments(j, k), 1, :), 1, length(Aux_X_dual)));
        set(F_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        ylabel('${[m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([F_separed_plot,F_dual_plot],{'$x_{baseline}$','$x_{dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        ylabel('${[m]}$','fontsize',11,'interpreter','latex', 'Color',C18);
        aux_title = string(experiments(j, k));
        title('Experiment ' + aux_title, 'fontsize', 11, 'interpreter', 'latex', 'Color', 'black');
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
        
        axes('Position',[dimension_x(k) dimension_y(j)-0.28 .25 .10]);
        %% Data generation
        M_separed_plot = line(t_separed,reshape(Aux_X_separed(experiments(j, k), 2, :), 1, length(Aux_X_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(Aux_X_dual(experiments(j, k), 2, :), 1, length(Aux_X_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        ylabel('${[m]}$','fontsize',8,'interpreter','latex', 'Color',C18);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$y_{baseline}$','$y_{dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        ylabel('${[m]}$','fontsize',11,'interpreter','latex', 'Color',C18);
        
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
        
        axes('Position',[dimension_x(k)  dimension_y(j)-0.43 .25 .10]);
        %% Data generation
        M_separed_plot = line(t_separed,reshape(Aux_X_separed(experiments(j, k), 3, :), 1, length(Aux_X_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(Aux_X_dual(experiments(j, k), 3, :), 1, length(Aux_X_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$z_{baseline}$','$z_{dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        ylabel('${[m]}$','fontsize',11,'interpreter','latex', 'Color',C18);
        xlabel('$\textrm{Time}[s]$','fontsize',11,'interpreter','latex','Color',C18);
        
        %% Figure properties
        ax_1 = gca;
        ax_1.Box = 'on';
        ax_1.BoxStyle = 'full';
        ax_1.TickLength = [0.01;0.01];
        %ax_1.XTickLabel = [];
        ax_1.TickDirMode = 'auto';
        ax_1.YMinorTick = 'on';
        ax_1.XMinorTick = 'on';
        ax_1.XMinorGrid = 'on';
        ax_1.YMinorGrid = 'on';
        ax_1.MinorGridAlpha = 0.15;
        ax_1.LineWidth = 0.8;
        ax_1.XLim = [t_dual(1), t_dual(end)]; % Set limits for x-axi
        
    end
end

set(gcf, 'Color', 'w'); % Sets axes background
name_control = "Positions_comparative_" + string(names) + ".pdf";
export_fig(name_control, '-pdf');

%% Quaternions
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;
for j=1:length(dimension_y)
    for k=1:length(dimension_x)
        axes('Position',[dimension_x(k) dimension_y(j)-0.13  .25 .10]);
        %% Data generation
        F_separed_plot = line(t_separed,reshape(Aux_X_separed(experiments(j, k), 4, :), 1, length(Aux_X_separed)));
        set(F_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        F_dual_plot = line(t_dual,reshape(Aux_X_dual(experiments(j, k), 4, :), 1, length(Aux_X_dual)));
        set(F_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([F_separed_plot,F_dual_plot],{'$q_{w-baseline}$','$q_{w-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        aux_title = string(experiments(j, k));
        title('Experiment ' + aux_title, 'fontsize', 11, 'interpreter', 'latex', 'Color', 'black');
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
        M_separed_plot = line(t_separed,reshape(Aux_X_separed(experiments(j, k), 5, :), 1, length(Aux_X_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(Aux_X_dual(experiments(j, k), 5, :), 1, length(Aux_X_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$q_{x-baseline}$','$q_{x-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
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
        M_separed_plot = line(t_separed,reshape(Aux_X_separed(experiments(j, k), 6, :), 1, length(Aux_X_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(Aux_X_dual(experiments(j, k), 6, :), 1, length(Aux_X_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        %xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$q_{y-baseline}$','$q_{y-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
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
        M_separed_plot = line(t_separed,reshape(Aux_X_separed(experiments(j, k), 7, :), 1, length(Aux_X_separed)));
        set(M_separed_plot, 'LineStyle', '-', 'Color', [C5, 0.5], 'LineWidth', 1.2*lw);
        M_dual_plot = line(t_dual,reshape(Aux_X_dual(experiments(j, k), 7, :), 1, length(Aux_X_dual)));
        set(M_dual_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
        
        %% Legend nomeclature
        hLegend_1 = legend([M_separed_plot,M_dual_plot],{'$q_{z-baseline}$','$q_{z-dual}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
        set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
        xlabel('$\textrm{Time}[s]$','fontsize',11,'interpreter','latex','Color',C18);
        
        %% Figure properties
        ax_1 = gca;
        ax_1.Box = 'on';
        ax_1.BoxStyle = 'full';
        ax_1.TickLength = [0.01;0.01];
        %ax_1.XTickLabel = [];
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
name_control = "Quaternions_comparative_" + string(names) + ".pdf";
export_fig(name_control, '-pdf');


%%%
end

