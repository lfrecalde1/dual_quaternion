%% PLot Quaternions
clc, clear all, close all;
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

Aux_X_dual = zeros(size(F_separed, 1), 7, length(X_dual));
Aux_X_separed = zeros(size(F_separed, 1), 7, length(X_separed));

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

C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;
%
dimension_x = [0.05, 0.38, 0.71];
dimension_y = [1.0, 0.5];
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
ig1_comps.fig = gcf;
colors = crameri('hawaii',size(F_separed, 1));

axes('Position',[dimension_x(1) dimension_y(1)-0.7  0.5 0.5]);
% Create a unit sphere
[x, y, z] = sphere(50);
surf(x, y, z, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
axis square;
axis equal;
light;
lighting phong;
camlight left;
alpha 0.1;  % Set transparency for better visualization
axis equal;
hold on;
set(gca,'ticklabelinterpreter','latex',...
            'fontsize',1.3*fontsizeTicks)
 title(['Initial quaternions DQ-NMPC'], 'FontSize', 12, 'Interpreter', 'latex', 'Color', 'black');
        
ax_1 = gca;
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;

for k=1:size(X_dual, 1)
%% Quaternion Separed
quaternion_dual = Aux_X_dual(k, 4:7, 1);

pt = rotatepoint(quaternion_dual, [1; 0; 0]);
% Extract the vector part of the quaternion
[Angle_axis(:, 1)] = quat2axang(quaternion_dual);


% Plot the quaternion vector on the sphere
quiver3(0, 0, 0, Angle_axis(1), Angle_axis(2), Angle_axis(3), 'Color', colors(k, :), 'LineWidth', 1.5*lw);
%quiver3(0, 0, 0, pt(1), pt(2), pt(3), 'Color', colors(k, :), 'LineWidth', 1.5*lw);

end

set(gcf, 'Color', 'w'); % Sets axes background
name_control = "Intit_quat.pdf";
export_fig(name_control, '-pdf');