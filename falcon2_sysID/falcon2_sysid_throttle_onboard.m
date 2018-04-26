clear all;
close all;
path(path, '../read_bags');
path(path, '../helper_functions');

% two experiments are needed to validate the identification
bagfile_exp =  '/data/rosbag/throttle_imu/throttle_imu_sample_8Ah.bag';

%% Select data in flight - remove stationary 
selection_start_time = 18;
selection_end_time = 119;

topic_imu = '/dji_sdk/imu';
topic_roll_pitch_yawrate_thrust = '/gc/roll_pitch_yawrate_thrust';

%mas of uav with 4Ah battery
%uav_mass = 2.096;
%mas of uav with 8Ah battery
uav_mass = 2.311;


bag = ros.Bag(bagfile_exp);

%%
% * Experiment info:*
bag.info

Experiment.IMU  = readImu(bag, topic_imu);
Experiment.RPYT  = readRollPitchYawRateThrottle(bag, topic_roll_pitch_yawrate_thrust);
%load('../rpyt2.mat') 

%% Calculate the Force using uav mass
Experiment.IMU.Force_z = Experiment.IMU.a(3,:) .* uav_mass;

%%
% * Plot Input Data

figure('Name','Original Data');
title('Experiment Original Data');
hold on;
yyaxis left
ylabel('Force_Z[N]');
plot(Experiment.IMU.t_relative, Experiment.IMU.Force_z, 'go--', 'linewidth', 1);

yyaxis right
ylabel('Throttle [%]');
plot(Experiment.RPYT.t_relative, Experiment.RPYT.thrust_z, 'o--', 'linewidth', 1);

xlabel('time');
legend('Force_Z','thrust');
grid on;




%% Interpolation of throttle data to match imu sampling points
thrust_z_imut=interp1(Experiment.RPYT.t,Experiment.RPYT.thrust_z,Experiment.IMU.t,'spline');

%this is system input
X_Force = Experiment.IMU.Force_z;
X_Time  = Experiment.IMU.t;

% This is system output
Y_Thrust = thrust_z_imut;
Y_Time   = Experiment.IMU.t;

[z,lag_z]=xcorr(Y_Thrust,X_Force);
z=medfilt1(z,46);
[~,I_z]=max(z);
time_sync_index=lag_z(I_z);

X_Force = circshift(X_Force, time_sync_index);
tsel = Experiment.IMU.t_relative >= selection_start_time & Experiment.IMU.t_relative < selection_end_time;


%%
% * Plot Input Data
figure('Name','Data after Synch');
title('Experiment Selected Data after sync');
hold on;
yyaxis left
ylabel('Force_Z[N]');
plot(X_Time, X_Force, 'go--', 'linewidth', 1);
plot(Experiment.IMU.t, tsel.*50, 'ro--', 'linewidth', 1);

yyaxis right
ylabel('Throttle [%]');
plot(Y_Time, Y_Thrust, 'o--', 'linewidth', 1);

xlabel('time');
legend('Force_Z', 'Selection','thrust');
grid on;



tsel = Experiment.IMU.t_relative >= selection_start_time & Experiment.IMU.t_relative < selection_end_time;
X_Force = X_Force(tsel);
X_Time = X_Time(tsel);
% Since we now use the same time sampling points as IMU (after interpolation) the tsel is the
% same as for IMU

Y_Thrust = Y_Thrust(tsel);
Y_Time = Y_Time(tsel);

%%
% * Plot Input Data
figure('Name','Selected Data');
title('Experiment Selected Data');
hold on;
yyaxis left
ylabel('Force_Z[N]');
plot(X_Time, X_Force, 'go--', 'linewidth', 1);

yyaxis right
ylabel('Throttle [%]');
plot(Y_Time, Y_Thrust, 'o--', 'linewidth', 1);

xlabel('time');
legend('Force_Z','thrust');
grid on;

p = polyfit(X_Force,Y_Thrust,1);
Y_ThrustFit = polyval(p, X_Force);

%%
% * Plot Input Data
figure('Name','Fited Output Data');
title('Experiment Fited Data');
hold on;
yyaxis left
ylabel('Force_Z[N]');
plot(X_Time, X_Force, 'go--', 'linewidth', 1);

yyaxis right
ylabel('Throttle [%]');
plot(Y_Time, Y_ThrustFit, 'ro--', 'linewidth', 1);
plot(Y_Time, Y_Thrust, 'bo--', 'linewidth', 1);

xlabel('time');
legend('Force', 'Thrust Fited','Thrust');
grid on;

disp(strcat('The Fitting linear model throttle = a * force + b; a=', num2str(p(1)), '; b=', num2str(p(2)), ';' ));


