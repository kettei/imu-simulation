close all
clc
clear

% -------------------------------------------------------------------------
% Folder Reading
addpath('WitMotion Data');
addpath('WitMotion Data/1');
addpath('WitMotion Data/1.1');
addpath('WitMotion Data/2'); %current
addpath('WitMotion Data/100 data rate');
addpath('WitMotion Data/100 100 2020');
addpath('WitMotion Data/2020');
addpath('WitMotion Data/11022020');
addpath('Quaternions Calculation');

% -------------------------------------------------------------------------
% D:\Files\THESIS\Desktop\Percobaan
% File reading
fid = fopen("Cobakotak.txt", "r");
cac = textscan(fid, '%d%c%d %{HH:mm:ss.SSS}D %f%f%f %f%f%f%f%f%f%f%f%f%f', "Headerlines",2);
fclose(fid);

samplePeriod = 1/200; %data rate per minute (huge impact on integration process) 200

% Convert acceleration units to m/s^2
accX = cac{5}*9.81; %all axis are affected by gravity
accY = cac{6}*9.81;
accZ = cac{7}*9.81;

%gyro opt
gyrX = cac{8};
gyrY = cac{9};
gyrZ = cac{10};

time = cac{4};
time = second(time);

% -------------------------------------------------------------------------
% Detect stationary periods

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HighPass filter accelerometer data
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LowPass filter accelerometer data
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% Threshold detection
stationary = acc_magFilt < 0.06; %0.06

% -------------------------------------------------------------------------
% Accelerometer Filter

% HighPass filter accelerometer data
% filtCutOff = 0.001;
% [c, d] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% [c, d] = butter(1, (2*filtCutOff)/(1/0.0050), 'high'); % 0.0050 = mean deltaTime
% accX = filtfilt(c, d, accX);
% accY = filtfilt(c, d, accY);
% accZ = filtfilt(c, d, accZ);

% LowPass filter accelerometer data
% filtCutOff = 5;
% [c, d] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% [c, d] = butter(1, (2*filtCutOff)/(1/0.0050), 'low'); % 0.0050 = mean deltaTime
% accX = filtfilt(c, d, accX);
% accY = filtfilt(c, d, accY);
% accZ = filtfilt(c, d, accZ);

% -------------------------------------------------------------------------
% Plot data raw sensor data and stationary periods

figure('Position', [210 65 900 600], 'NumberTitle', 'off', 'Name', 'Sensor (Acc/Gyr) Data');
ax(1) = subplot(2,1,2);
    hold on;
    plot(time, gyrX, 'r');
    plot(time, gyrY, 'g');
    plot(time, gyrZ, 'b');
    title('Gyroscope');
    xlabel('Time (s)');
    ylabel('Angular velocity (^\rad/s)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(2,1,1);
    hold on;
    plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');
    plot(time, acc_magFilt, ':k');
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s2)');
    legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    legend('X', 'Y', 'Z', 'Filtered');
    hold off;
linkaxes(ax,'x');

% -------------------------------------------------------------------------
% Compute orientation

quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);

% For all data
for t = 1:length(time)
    if(stationary(t))
        AHRSalgorithm.Kp = 1;
    else
        AHRSalgorithm.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
end

% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));

% % Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
hold on;
plot(time, acc(:,1), 'r');
plot(time, acc(:,2), 'g');
plot(time, acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod; %delta time
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0]; % force zero velocity when stationary
    end
end

% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Remove integral drift
vel = vel - velDrift;

% % Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational position

% Integrate velocity to yield position
pos = zeros(size(vel));
% pos = cumtrapz(deltaTime, vel);
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % integrate velocity to gain position
end

% % Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off; 

% -------------------------------------------------------------------------
% 3D Plot

posPlot = pos;
quatPlot = quat;

% Create 6 DOF animation create AVI = False
SamplePlotFreq = 5;
Spin = 100;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [250 35 860 640], 'View', [(20:(Spin/(length(posPlot)-1)):(20+Spin))', 20*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq)); %mean deltaTime
