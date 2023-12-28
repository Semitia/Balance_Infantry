% MATLAB controller for Webots
% File:          balanced_robot_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
keyboard;

TIME_STEP = 1; % 1ms

% params

%%% motor params
%joint motor
max_joint_motor_torque = 100;
% wheel motor
max_wheel_motor_torque = 30;

% LQR Params
K = getLQRParam();

% sensors init

% Pitch
imu = wb_robot_get_device('imu');
wb_inertial_unit_enable(imu, TIME_STEP);

accelerometer = wb_robot_get_device('Acce');
wb_accelerometer_enable(accelerometer, TIME_STEP);

% motor
motors(1).handle = wb_robot_get_device('RBM');
motors(2).handle = wb_robot_get_device('RFM');
motors(3).handle = wb_robot_get_device('wheel1');
motors(4).handle = wb_robot_get_device('LBM');
motors(5).handle = wb_robot_get_device('LFM');
motors(6).handle = wb_robot_get_device('wheel2');

% torque feedback
for i = 1:6
    wb_motor_enable_torque_feedback(motors(i).handle, TIME_STEP);
end

% max motor torque
for i = 1:6

    if i == 3 || i == 6
        wb_motor_set_available_torque(motors(i).handle, max_wheel_motor_torque);
    else
        wb_motor_set_available_torque(motors(i).handle, max_joint_motor_torque);
    end

end

% pos sensors
sensors(1).handle = wb_robot_get_device('RBS');
sensors(2).handle = wb_robot_get_device('RFS');
sensors(3).handle = wb_robot_get_device('Swheel1');
sensors(4).handle = wb_robot_get_device('LBS');
sensors(5).handle = wb_robot_get_device('LFS');
sensors(6).handle = wb_robot_get_device('Swheel2');

for i = 1:6
    wb_position_sensor_enable(sensors(i).handle, TIME_STEP);
end

%gps
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

%gyro
gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

%keybord control
wb_keyboard_enable(TIME_STEP);

% sensors read info

time = 0;
imu_info = [];
gps_info = [];
gyro_info = [];
acc_info = [];
key_info = 0;

%%% pos sensors
for i = 1:6
    sensors(i).pos = 0;
    sensors(i).pos_last = 0;
    sensors(i).w = 0;
end

%%% motor torque info
for i = 1:6
    motors(i).torque = [];
end

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1

    % read the sensors
    % time
    time = wb_robot_get_time();
    % gps
    gps_info = wb_gps_get_values(gps);

    % accelerometer
    acc_info = wb_accelerometer_get_values(accelerometer);

    % gyro
    gyro_info = wb_gyro_get_values(gyro);

    % imu
    imu_info = wb_inertial_unit_get_roll_pitch_yaw(imu);

    % keyboard
    key_info = wb_keyboard_get_key();

    % motors
    for i = 1:6
        motors(i).torque = wb_motor_get_torque_feedback(motors(i).handle);
    end

    % sensors
    for i = 1:6
        sensors(i).pos = wb_position_sensor_get_value(sensors(i).handle);
        sensors(i).w = sensors(i).pos - sensors(i).pos_last; %注意单位
        sensors(i).pos_last = sensors(i).pos;
    end

    % Process here sensor data

    % send actuator commands

    % code plots some graphics

    %drawnow;

end

% cleanup code goes here: write data to files, etc.
