%% Clean data
clc;clear;close all

%% Load Rosbag 
rosbag_file= rosbag('2023-09-02-03-49-17.bag');

%% Specify the name of one of the topics
odometry_topic = select(rosbag_file,'Topic','/dji_sdk/odometry'); 
odometry_struc = readMessages(odometry_topic,'DataFormat','struct');

%% Entrada de datos de odometria

for i = 1: size(odometry_struc,1)
    nano_Sec = (odometry_struc{i, 1}.Header.Stamp.Nsec) / 1000000000 
    time_odometry(i) = odometry_struc{i, 1}.Header.Stamp.Sec + nano_Sec;

    x(1,i) = odometry_struc{i,1}.Pose.Pose.Position.X;
    x(2,i) = odometry_struc{i,1}.Pose.Pose.Position.Y;
    x(3,i) = odometry_struc{i,1}.Pose.Pose.Position.Z;
    
    quat(1,i) = odometry_struc{i,1}.Pose.Pose.Orientation.X;
    quat(2,i) = odometry_struc{i,1}.Pose.Pose.Orientation.Y;
    quat(3,i) = odometry_struc{i,1}.Pose.Pose.Orientation.Z;
    quat(4,i) = odometry_struc{i,1}.Pose.Pose.Orientation.W;
    
    euler_aux = (quat2eul([quat(4,i) quat(1,i) quat(2,i) quat(3,i)],'ZYX'))';
    euler(:,i) = [euler_aux(3);euler_aux(2);euler_aux(1)];
    
    x_p(1,i) = odometry_struc{i,1}.Twist.Twist.Linear.X;
    x_p(2,i) = odometry_struc{i,1}.Twist.Twist.Linear.Y;
    x_p(3,i) = odometry_struc{i,1}.Twist.Twist.Linear.Z;
    
    omega(1,i) = odometry_struc{i,1}.Twist.Twist.Angular.X;
    omega(2,i) = odometry_struc{i,1}.Twist.Twist.Angular.Y;
    omega(3,i) = odometry_struc{i,1}.Twist.Twist.Angular.Z;
end


%% Entrada de datos ( es otro topic con otros sampleos al de arriba) 

inputs_topic = select(rosbag_file,'Topic','/m100/velocityControl');
inputs_struc = readMessages(inputs_topic,'DataFormat','struct');

time_inputs =  zeros(size(inputs_struc));

v_ref =  zeros(size(inputs_struc));


for i = 1: size(inputs_struc,1)
    nano_Sec = (inputs_struc{i, 1}.Header.Stamp.Nsec) / 1000000000  ;
    time_inputs(i) = inputs_struc{i, 1}.Header.Stamp.Sec + nano_Sec;

    v_ref(1,i) = inputs_struc{i,1}.Twist.Linear.Z;
    v_ref(2,i) = inputs_struc{i,1}.Twist.Angular.X;
    v_ref(3,i) = inputs_struc{i,1}.Twist.Angular.Y;
    v_ref(4,i) = inputs_struc{i,1}.Twist.Angular.Z;

end

figure
plot(time_odometry,x(1,:));hold on
figure
plot(time_inputs,v_ref(1,:));hold on

