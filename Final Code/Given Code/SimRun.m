clear; clc; close all;

%% Controller Parameter Definitions
run Geometry.m; % creates a linkList in the workspace for you and for the simulation
load points3D; % loads the CSM trajectory points

%% Define Kp and Kd gains
Kp = 0; 
Kd = 0; 

% provides an initial target to see the simulation work
[t1,t2,t3,t4,t5,t6] = abbInvKine([eye(3) points3D(1,:)';0 0 0 1],[0;-pi/2;0;0;0;0]);
target = [t1,t2,t3,t4,t5,t6]';

%% Choose Simulation System (perfect model or realistic model)
Sim_Exact = true; % sets if the simulated geometry is exact (true) or slightly different (false)

%% Enable/Disable Control
control_enable = true;
set_param('Project3_System/control_enable', 'sw', int2str(control_enable))

%% Run Simulation
simTime = 15;
simOut =  sim('Project3_System','SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');


%% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');

%% Plot theta as a function of time
figure(1)
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end

%% Display Arm Motion Movie
figure(2)
plot(0,0); ah = gca; % just need a current axis handel
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
   plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
   hold on;
   plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
         reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
         reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
   plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
   hold off;
   pause(1/30)
end

