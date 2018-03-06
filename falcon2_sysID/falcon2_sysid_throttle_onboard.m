clear all
path(path, '../read_bags');
path(path, '../helper_functions');

% two experiments are needed to validate the identification
bagfile_exp =  '/data/rosbag/throttle_imu/throttle_imu_sample_4Ah_01.bag';

topic_imu = '/dji_sdk/imu';
topic_roll_pitch_yawrate_thrust = '/gc/roll_pitch_yawrate_thrust';

bag = ros.Bag(bagfile_exp);

%%
% * Experiment info:*
bag.info

Experiment.IMU  = readImu(bag, topic_imu);
Experiment.rpy_imu = quat2rpy(Experiment.IMU.q);
Experiment.RPYT  = readRollPitchYawRateThrottle(bag, topic_roll_pitch_yawrate_thrust);

figure;
title('Experiment Data');
plot(Experiment.IMU.t, Experiment.IMU.a(3, :), ...
    Experiment.RPYT.t, Experiment.RPYT.thrust_z, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('acceleration','thrust');
ylabel('m/s^2');
