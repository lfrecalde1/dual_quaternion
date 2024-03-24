%% Code to get the data during the experiment.
clc, clear all, close all;

%% Load Rosbag
bag = rosbag("2023-09-06-21-14-37.bag");

%% Get topic Data 
bag_topic_error_velocity = select(bag,'Time',[bag.StartTime bag.EndTime],'Topic','/dji_sdk/visual_servoing/errores');
%% Get Data Errors frontal velocity and altitude
data_error_and_velocity = timeseries(bag_topic_error_velocity,'Twist.Linear.X', 'Twist.Linear.Y', 'Twist.Linear.Z', 'Twist.Angular.X', 'Twist.Angular.Y',  'Twist.Angular.Z');
time_e = data_error_and_velocity.Time;
delta_time_e = diff(time_e);


%% Get Data Control values
bag_topic_velocity_c = select(bag,'Time',[bag.StartTime bag.EndTime],'Topic','/m100/velocityControl');
data_velocity_control = timeseries(bag_topic_velocity_c,'Twist.Linear.X', 'Twist.Linear.Y', 'Twist.Linear.Z', 'Twist.Angular.X', 'Twist.Angular.Y',  'Twist.Angular.Z');
time_v = data_velocity_control.Time;
delta_time_v = diff(time_v);
% 
% %% Get Data real Velocities values
bag_topic_velocity_r = select(bag,'Time',[bag.StartTime bag.EndTime],'Topic','/dji_sdk/odometry');
data_velocity_real = timeseries(bag_topic_velocity_r,'Twist.Twist.Linear.X', 'Twist.Twist.Linear.Y', 'Twist.Twist.Linear.Z', 'Twist.Twist.Angular.X', 'Twist.Twist.Angular.Y',  'Twist.Twist.Angular.Z');
data_pose_real = timeseries(bag_topic_velocity_r,'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', 'Pose.Pose.Position.Z', 'Pose.Pose.Orientation.W',  'Pose.Pose.Orientation.X',   'Pose.Pose.Orientation.Y', 'Pose.Pose.Orientation.Z');
time_o = data_velocity_real.Time;
delta_time_o = diff(time_o);

%% New time vector defintion
t_e(1) = 0;
t_v(1) = 0;
t_o(1) = 0';

%% Data Values
error_r = data_error_and_velocity.Data(:,3)';
error_phi = data_error_and_velocity.Data(:,2)';
vx_d = data_error_and_velocity.Data(:,5)';
z_d = data_error_and_velocity.Data(:,4)';
z_d = z_d*(2.9365/300);

%% Control velocities of the system
v_c =  data_velocity_control.Data(:, 1:3)';
w_c =  data_velocity_control.Data(:, 4:6)';

%% Real velocitiesof the system
vw_r = data_velocity_real.Data(:, 1:3)';
ww_r = data_velocity_real.Data(:, 4:6)';
quatw_r = data_pose_real.Data(:, 4:7)';
Hw_r = data_pose_real.Data(:, 1:3)';

%% Get velocities frame B
for k = 1:length(vw_r)
   euler_r=  (quat2eul([quatw_r(1, k) quatw_r(2, k) quatw_r(3, k) quatw_r(4, k)],'ZYX'))';
   euler_real(:, k) = [euler_r(3);euler_r(2);euler_r(1)];
end


%% Get velocities body frame
for k = 1:length(vw_r)
   vb_r(:, k) = inv(Rot_zyx(euler_real(:, k)))*vw_r(:, k);
end


for k=1:length(delta_time_e)
    %% Set Vector Time
    t_e(k+1) = t_e(k) + delta_time_e(k);
end

for k=1:length(delta_time_v)
    %% Set Vector Time
    t_v(k+1) = t_v(k) + delta_time_v(k);
end

for k = 1:length(delta_time_o)
    t_o(k+1) = t_o(k) + delta_time_o(k);
end

index_zoom_1 = (t_e>=0) & (t_e<=40);
index_zoom_2 = (t_o>=0) & (t_o<=40);

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

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[0.05  0.31 .44 .19]);
%% Data generation
error_x_plot = line(t_e(index_zoom_1),error_r(index_zoom_1));
set(error_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
error_y_plot = line(t_e(index_zoom_1),error_phi(index_zoom_1));
set(error_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
ylabel('${[pixels, rad]}$','fontsize',10,'interpreter','latex', 'Color',C18);
%% Legend nomeclature
hLegend_1 = legend([error_x_plot,error_y_plot],{'$~^i \tilde{r}$','$~^i \tilde{\theta}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
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

%% PLot 2
axes('Position',[0.05 0.75 .44 .19]);
%% Data generation
v_x_plot = line(t_e(index_zoom_1),v_c(1,index_zoom_1));
set(v_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
v_y_plot = line(t_e(index_zoom_1),v_c(2,index_zoom_1));
set(v_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
v_z_plot = line(t_e(index_zoom_1),v_c(3,index_zoom_1));
set(v_z_plot, 'LineStyle', '-', 'Color', [70, 110, 155]/255, 'LineWidth', 1.1*lw);
hTitle_2 = title({'$\textrm{MPC Results}$'},'fontsize',14,'interpreter','latex','Color',C18);
ylabel('${[m/s]}$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([v_x_plot,v_y_plot, v_z_plot],{'$~^b{\upsilon_{xc}}$','$~^b{\upsilon_{yc}}$','$~^b{\upsilon_{zc}}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.3*fontsizeTicks)
%% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.XTickLabel = [];
ax_2.TickLength = [0.01;0.01];
ax_2.TickDirMode = 'auto';
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;

%% PLot 3
axes('Position',[0.05 0.53 .44 .19]);
%% Data generation
w_x_plot = line(t_e(index_zoom_1),w_c(1,index_zoom_1));
set(w_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
w_y_plot = line(t_e(index_zoom_1),w_c(2,index_zoom_1));
set(w_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
w_z_plot = line(t_e(index_zoom_1),w_c(3,index_zoom_1));
set(w_z_plot, 'LineStyle', '-', 'Color', [70, 110, 155]/255, 'LineWidth', 1.1*lw);
ylabel('${[rad/s]}$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([w_x_plot,w_y_plot, w_z_plot],{'$~^b{{\omega}_{xc}}$','$~^b{{\omega}_{yc}}$','$~^b{{\omega}_{zc}}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.3*fontsizeTicks)
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
ax_3.XTickLabel = [];
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;

axes('Position',[0.05  0.09 .44 .19]);
%% Data generation
error_vx_plot = line(t_o(index_zoom_1), vx_d(1, index_zoom_1)-vb_r(1, index_zoom_1));
set(error_vx_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
error_z_plot = line(t_o(index_zoom_1), z_d(1, index_zoom_1) - Hw_r(3, index_zoom_1));
set(error_z_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('${[m/s, m]}$','fontsize',10,'interpreter','latex', 'Color',C18);
%% Legend nomeclature
hLegend_1 = legend([error_vx_plot, error_z_plot],{'$^{b}{\upsilon}_{xd}-~^b{\upsilon_{x}}$','$^w\eta_{zd} - ~^w\eta_z$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.3*fontsizeTicks);
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
set(gcf, 'Color', 'w'); % Sets axes background
export_fig Control_errors_apropiate.pdf -q101

% %% New Experiment  Data
% clear all
% %% Load Rosbag
% bag = rosbag("2023-09-06-04-40-52.bag");
% bag_topic_error_velocity = select(bag,'Time',[bag.StartTime bag.EndTime],'Topic','/dji_sdk/visual_servoing/errores');
% %% Get Data Errors frontal velocity and altitude
% data_error_and_velocity = timeseries(bag_topic_error_velocity,'Twist.Linear.X', 'Twist.Linear.Y', 'Twist.Linear.Z', 'Twist.Angular.X', 'Twist.Angular.Y',  'Twist.Angular.Z');
% time_e = data_error_and_velocity.Time;
% delta_time_e = diff(time_e);
% 
% 
% %% Get Data Control values
% bag_topic_velocity_c = select(bag,'Time',[bag.StartTime bag.EndTime],'Topic','/m100/velocityControl');
% data_velocity_control = timeseries(bag_topic_velocity_c,'Twist.Linear.X', 'Twist.Linear.Y', 'Twist.Linear.Z', 'Twist.Angular.X', 'Twist.Angular.Y',  'Twist.Angular.Z');
% time_v = data_velocity_control.Time;
% delta_time_v = diff(time_v);
% % 
% % %% Get Data real Velocities values
% bag_topic_velocity_r = select(bag,'Time',[bag.StartTime bag.EndTime],'Topic','/dji_sdk/odometry');
% data_velocity_real = timeseries(bag_topic_velocity_r,'Twist.Twist.Linear.X', 'Twist.Twist.Linear.Y', 'Twist.Twist.Linear.Z', 'Twist.Twist.Angular.X', 'Twist.Twist.Angular.Y',  'Twist.Twist.Angular.Z');
% data_pose_real = timeseries(bag_topic_velocity_r,'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', 'Pose.Pose.Position.Z', 'Pose.Pose.Orientation.W',  'Pose.Pose.Orientation.X',   'Pose.Pose.Orientation.Y', 'Pose.Pose.Orientation.Z');
% time_o = data_velocity_real.Time;
% delta_time_o = diff(time_o);
% 
% %% New time vector defintion
% t_e(1) = 0;
% t_v(1) = 0;
% t_o(1) = 0';
% 
% %% Data Values
% error_r = data_error_and_velocity.Data(:,3)';
% error_phi = data_error_and_velocity.Data(:,2)';
% vx_d = data_error_and_velocity.Data(:,5)';
% z_d = data_error_and_velocity.Data(:,4)';
% z_d = z_d*(2.9365/300);
% 
% %% Control velocities of the system
% v_c =  data_velocity_control.Data(:, 1:3)';
% w_c =  data_velocity_control.Data(:, 4:6)';
% 
% %% Real velocitiesof the system
% vw_r = data_velocity_real.Data(:, 1:3)';
% ww_r = data_velocity_real.Data(:, 4:6)';
% quatw_r = data_pose_real.Data(:, 4:7)';
% Hw_r = data_pose_real.Data(:, 1:3)';
% 
% %% Get velocities frame B
% for k = 1:length(vw_r)
%    euler_r=  (quat2eul([quatw_r(1, k) quatw_r(2, k) quatw_r(3, k) quatw_r(4, k)],'ZYX'))';
%    euler_real(:, k) = [euler_r(3);euler_r(2);euler_r(1)];
% end
% 
% 
% %% Get velocities body frame
% for k = 1:length(vw_r)
%    vb_r(:, k) = inv(Rot_zyx(euler_real(:, k)))*vw_r(:, k);
% end
% 
% 
% for k=1:length(delta_time_e)
%     %% Set Vector Time
%     t_e(k+1) = t_e(k) + delta_time_e(k);
% end
% 
% for k=1:length(delta_time_v)
%     %% Set Vector Time
%     t_v(k+1) = t_v(k) + delta_time_v(k);
% end
% 
% for k = 1:length(delta_time_o)
%     t_o(k+1) = t_o(k) + delta_time_o(k);
% end
% 
% index_zoom_1 = (t_e>=0) & (t_e<=40);
% index_zoom_2 = (t_o>=0) & (t_o<=40);
% 
% %% Figures Colors Definition
% lw = 1; % linewidth 1
% lwV = 2; % linewidth 2zz
% fontsizeLabel = 11; %11
% fontsizeLegend = 11;
% fontsizeTicks = 11;
% fontsizeTitel = 11;
% sizeX = 1300; % size figure
% sizeY = 1000; % size figure
% 
% 
% % color propreties
% c1 = [80, 81, 79]/255;
% c2 = [244, 213, 141]/255;
% c3 = [242, 95, 92]/255;
% c4 = [112, 141, 129]/255;
% 
% C18 = [0 0 0];
% c5 = [130, 37, 37]/255;
% c6 = [205, 167, 37]/255;
% c7 = [81, 115, 180]/255;
% 
% C1 = [246 170 141]/255;
% C2 = [51 187 238]/255;
% C3 = [0 153 136]/255;
% C4 = [238 119 51]/255;
% C5 = [204 51 17]/255;
% C6 = [238 51 119]/255;
% C7 = [187 187 187]/255;
% C8 = [80 80 80]/255;
% C9 = [140 140 140]/255;
% C10 = [0 128 255]/255;
% C11 = [234 52 89]/255;
% C12 = [39 124 252]/255;
% C13 = [40 122 125]/255;
% %C14 = [86 215 219]/255;
% C14 = [252 94 158]/255;
% C15 = [244 171 39]/255;
% C16 = [100 121 162]/255;
% C17 = [255 0 0]/255;
% 
% figure('Position', [500 500 sizeX sizeY])
% set(gcf, 'Position', [500 500 sizeX sizeY]);
% fig1_comps.fig = gcf;
% 
% axes('Position',[0.53  0.31 .44 .19]);
% %% Data generation
% error_x_plot = line(t_e(index_zoom_1),error_r(index_zoom_1));
% set(error_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
% error_y_plot = line(t_e(index_zoom_1),error_phi(index_zoom_1));
% set(error_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
% %% Legend nomeclature
% hLegend_1 = legend([error_x_plot,error_y_plot],{'$~^i \tilde{r}$','$~^i \tilde{\theta}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
%  set(gca,'ticklabelinterpreter','latex',...
%          'fontsize',1.3*fontsizeTicks)
% %% Figure properties
% ax_1 = gca;
% ax_1.Box = 'on';
% ax_1.BoxStyle = 'full';
% ax_1.TickLength = [0.01;0.01];
% ax_1.XTickLabel = [];
% ax_1.TickDirMode = 'auto';
% ax_1.YMinorTick = 'on';
% ax_1.XMinorTick = 'on';
% ax_1.XMinorGrid = 'on';
% ax_1.YMinorGrid = 'on';
% ax_1.MinorGridAlpha = 0.15;
% ax_1.LineWidth = 0.8;
% 
% %% PLot 2
% axes('Position',[0.53 0.75 .44 .19]);
% %% Data generation
% v_x_plot = line(t_e(index_zoom_1),v_c(1,index_zoom_1));
% set(v_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
% v_y_plot = line(t_e(index_zoom_1),v_c(2,index_zoom_1));
% set(v_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
% v_z_plot = line(t_e(index_zoom_1),v_c(3,index_zoom_1));
% set(v_z_plot, 'LineStyle', '-', 'Color', [70, 110, 155]/255, 'LineWidth', 1.1*lw);
% hTitle_2 = title({'$\textrm{Inappropriate selection of } \mathbf{W}$'},'fontsize',14,'interpreter','latex','Color',C18);
% 
% %% Legend nomeclature
% hLegend_2 = legend([v_x_plot,v_y_plot, v_z_plot],{'$~^b{\upsilon_{xc}}$','$~^b{\upsilon_{yc}}$','$~^b{\upsilon_{zc}}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
%  set(gca,'ticklabelinterpreter','latex',...
%          'fontsize',1.3*fontsizeTicks)
% %% Figure properties
% ax_2 = gca;
% ax_2.Box = 'on';
% ax_2.BoxStyle = 'full';
% ax_2.XTickLabel = [];
% ax_2.TickLength = [0.01;0.01];
% ax_2.TickDirMode = 'auto';
% ax_2.YMinorTick = 'on';
% ax_2.XMinorTick = 'on';
% ax_2.XMinorGrid = 'on';
% ax_2.YMinorGrid = 'on';
% ax_2.MinorGridAlpha = 0.15;
% ax_2.LineWidth = 0.8;
% 
% %% PLot 3
% axes('Position',[0.53 0.53 .44 .19]);
% %% Data generation
% w_x_plot = line(t_e(index_zoom_1),w_c(1,index_zoom_1));
% set(w_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
% w_y_plot = line(t_e(index_zoom_1),w_c(2,index_zoom_1));
% set(w_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
% w_z_plot = line(t_e(index_zoom_1),w_c(3,index_zoom_1));
% set(w_z_plot, 'LineStyle', '-', 'Color', [70, 110, 155]/255, 'LineWidth', 1.1*lw);
% 
% %% Legend nomeclature
% hLegend_3 = legend([w_x_plot,w_y_plot, w_z_plot],{'$~^b{{\omega}_{xc}}$','$~^b{{\omega}_{yc}}$','$~^b{{\omega}_{zc}}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
%  set(gca,'ticklabelinterpreter','latex',...
%          'fontsize',1.3*fontsizeTicks)
% %% Figure properties
% ax_3 = gca;
% ax_3.Box = 'on';
% ax_3.BoxStyle = 'full';
% ax_3.XTickLabel = [];
% ax_3.TickLength = [0.01;0.01];
% ax_3.TickDirMode = 'auto';
% ax_3.YMinorTick = 'on';
% ax_3.XMinorTick = 'on';
% ax_3.XMinorGrid = 'on';
% ax_3.YMinorGrid = 'on';
% ax_3.MinorGridAlpha = 0.15;
% ax_3.LineWidth = 0.8;
% 
% axes('Position',[0.53  0.09 .44 .19]);
% %% Data generation
% error_vx_plot = line(t_o(index_zoom_1), vx_d(1, index_zoom_1)-vb_r(1, index_zoom_1));
% set(error_vx_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
% error_z_plot = line(t_o(index_zoom_1), z_d(1, index_zoom_1) - Hw_r(3, index_zoom_1));
% set(error_z_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
% xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
% %% Legend nomeclature
% hLegend_1 = legend([error_vx_plot, error_z_plot],{'$^{b}{\upsilon}_{xd}-~^b{\upsilon_{x}}$','$^w\eta_{zd} - ~^w\eta_z$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
%  set(gca,'ticklabelinterpreter','latex',...
%          'fontsize',1.3*fontsizeTicks);
% %% Figure properties
% ax_1 = gca;
% ax_1.Box = 'on';
% ax_1.BoxStyle = 'full';
% ax_1.TickLength = [0.01;0.01];
% ax_1.TickDirMode = 'auto';
% ax_1.YMinorTick = 'on';
% ax_1.XMinorTick = 'on';
% ax_1.XMinorGrid = 'on';
% ax_1.YMinorGrid = 'on';
% ax_1.MinorGridAlpha = 0.15;
% ax_1.LineWidth = 0.8;
% set(gcf, 'Color', 'w'); % Sets axes background
% export_fig Control_errors_inapropiate.pdf -q101