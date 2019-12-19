clear; clc; close all;

%% Controller Parameter Definitions
run Geometry.m; % creates a linkList in the workspace for you and for the simulation
load points3D; % loads the CSM trajectory points

%% Define Kp and Kd gains
Kp = 1000; 
Kd = 150; 

% Convert target points from xyz to theta-space
[normal,~,~] = affine_fit(points3D);
T = [zeros(3,4); 0, 0, 0, 1];
target = zeros(length(points3D)+2,6);

target(1,:) = [0;pi/2;0;0;0;0]';
dist = (points3D(2,:)-points3D(1,:))';

x_u = dist/norm(dist);
z_u = normal;
y_u = cpMap(z_u)*x_u;
T(1:3,1:3) = [x_u, y_u, z_u];
T(1:3,4) = points3D(1,:)';
[t1,t2,t3,t4,t5,t6,~] = abbInvKine(T, target(1,:)');
target(2,:) = [t1,t2,t3,t4,t5,t6];
target(3,:) = target(2,:);
for i = 1:length(points3D)-1
    dist = (points3D(i+1,:)-points3D(i,:))';
    x_u = dist/norm(dist);
    z_u = normal;
    y_u = cpMap(z_u)*x_u;
    T(1:3,1:3) = [x_u, y_u, z_u];
    T(1:3,4) = points3D(i+1,:)';
    [t1,t2,t3,t4,t5,t6,~] = abbInvKine(T, target(i+2,:)');
    target(i+3,:) = [t1,t2,t3,t4,t5,t6];
end

t_max = 5.5;

th_traj = target;
traj_mat_size = size(th_traj);
trajectory = zeros(traj_mat_size(1), traj_mat_size(2)+1);

t_traj0 = 0;
t_traj1 = 2;
t_traj = linspace(2.5,t_max,21)';
t_traj = [t_traj0; t_traj1; t_traj];

trajectory(:,1) = t_traj;
trajectory(1:end,2:end) = th_traj(1:end,:);

%% Choose Simulation System (perfect model or realistic model)
Sim_Exact = true; % sets if the simulated geometry is exact (true) or slightly different (false)

%% Enable/Disable Control
control_enable = true;
set_param('Project3_Part1_sim/control_enable', 'sw', int2str(control_enable))

%% Run Simulation
simTime = 15;
simOut =  sim('Project3_Part1_sim','SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');


%% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_dot_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');
pos_err = simOut.get('pos_err');

%% Plot theta as a function of time
figure(1)
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)],'Fontsize', 12)
    xlabel('time (s)','Fontsize', 12)
    ylabel('angle (rad)','Fontsize', 12)
    grid on;
end

%% Plot theta_dot as a function of time
figure(2)
for i=1:6
    subplot(3,2,i)
    plot(theta_dot_actual.time,theta_dot_actual.signals.values(:,i))
    title(['d\theta_', int2str(i),'/dt'],'Fontsize', 12)
    xlabel('time (s)','Fontsize', 12)
    ylabel('angle (rad)','Fontsize', 12)
    grid on;
end

%% Display Arm Motion Movie
figure(3)
plot(0,0); ah = gca; % just need a current axis handel
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
   plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
   xlabel('x (m)','Fontsize', 12)
   ylabel('y (m)','Fontsize', 12)
   zlabel('z (m)','Fontsize', 12)
   title('Manipulatory Trajectory vs Given Trajectory','Fontsize', 12)
   hold on;
   plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
         reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
         reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
   plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
   hold off;
   pause(1/30)
end

%% Plot XYZ error as a function of time
RMS_pos_err = zeros(length(pos_err.signals.values(:,1)),3);

x_err = pos_err.signals.values(:,1);
RMS_pos_err(:,1) = sqrt(x_err.^2);
y_err = pos_err.signals.values(:,2);
RMS_pos_err(:,2) = sqrt(y_err.^2);
z_err = pos_err.signals.values(:,3);
RMS_pos_err(:,3) = sqrt(z_err.^2);

figure(4)
for i=1:3

    subplot(3,1,i)
    plot(pos_err.time,RMS_pos_err(:,i))
    
    if i == 1
        title('Error_x = x_d - x_a','Fontsize', 12)
        ylabel('Error_x (m)','Fontsize', 12)
    elseif i == 2
        title('Error_y = y_d - y_a','Fontsize', 12)
        ylabel('Error_y (m)','Fontsize', 12)
    elseif i == 3
        title('Error_z = z_d - z_a','Fontsize', 12)
        ylabel('Error_z (m)','Fontsize', 12)
    end
    xlabel('time (s)','Fontsize', 12)
    grid on;
end

err_min = min(RMS_pos_err)
err_max = max(RMS_pos_err)
err_avg = mean(RMS_pos_err)
%% Plot torque as a function of time
figure(5)
for i=1:6
    subplot(3,2,i)
    plot(control_torque.time,control_torque.signals.values(:,i))
    title(['Control Torque at \theta_', int2str(i)],'Fontsize', 12)
    xlabel('time (s)','Fontsize', 12)
    ylabel('Torque (Nm)','Fontsize', 12)
    grid on;
end

torque_min = min(control_torque.signals.values)
torque_max = max(control_torque.signals.values)

%% Display Arm Motion Movie
figure(6)
plot(0,0); ah = gca; % just need a current axis handel
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
   camproj('perspective')
   daspect([1 1 1])
   set(gca,'BoxStyle','full','Box','on')
   grid on;
   hold on;
   xlabel('x (m)','Fontsize', 12)
   ylabel('y (m)','Fontsize', 12)
   zlabel('z (m)','Fontsize', 12)
   title('Manipulatory Trajectory vs Given Trajectory','Fontsize', 12)
   plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
         reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
         reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
   plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
   hold off;
   pause(1/30)
end