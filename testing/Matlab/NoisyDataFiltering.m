%% Initialization of Data
addpath('DataDumps\NoisyDataFiltering');

%Pull in acceleration data
ax = csvread('ax_noisy.csv',0,1);
ay = csvread('ay_noisy.csv',0,1);
az = csvread('az_noisy.csv',0,1);

accel = [ax, ay, az];
clear ax ay az;

%Pull in gyroscope data 
gx = csvread('gx_noisy.csv',0,1);
gy = csvread('gy_noisy.csv',0,1);
gz = csvread('gz_noisy.csv',0,1);

gyro = [gx, gy, gz];
clear gx gy gz;

%% Setup of UKF 
plotRawData = 1;
plotUKFData = 1;

% Initial State Estimate: X (Nx1)
X = [0 0 0 0 0 0]'; 

% Initial Uncertainty (Covariance): P (NxN)
[pax,pay,paz] = deal(0.5);  %m/s^2
[pgx,pgy,pgz] = deal(8.0);  %dps  

P = diag([pax^2, pay^2, paz^2, pgx^2, pgy^2, pgz^2]);

% State Transition Matrix: Phi (NxN)
Phi = eye(6); %Identity because prediction is also measurement

% Measurement Selection: H (NxN)
H = eye(6);

% Process Noise: Q (NxN)
unit_q = [5.00e-7, 1.25e-5, 1.66e-4;
          1.25e-5, 3.33e-4, 5.00e-3;
          1.66e-4, 5.00e-3, 1.00e-1];
      
Q = blkdiag(unit_q, unit_q);
clear unit_q;

% Measurement Uncertainty (Covariance): R (NxN)
[rax,ray,raz] = deal(0.5); %m/s^2
[rgx,rgy,rgz] = deal(0.8); %dps

R = diag([rax^2, ray^2, raz^2, rgx^2, rgy^2, rgz^2]);

% Input Transition Matrix: B (NxN)
B = zeros(6);

% Concatenate the input data
measured_data = [accel'; gyro'];

%% Run the UKF Algorithm
Xout = UnscentedKalmanFilter(measured_data,X,Phi,B,measured_data,P,Q,R,H,[0.5,2,1]);

%% Plot the UKF Data
if plotUKFData
    figure(1); clf(1); 
    
    % Acceleration in X
    subplot(3,1,1); hold on; 
    plot(Xout(1,:), 'b', 'LineWidth', 2);
    plot(measured_data(1,:), '--');
    
    title('UKF Acceleration Data X');
    xlabel('Data Points');
    ylabel('Acceleration (m/s^2)');
    legend('Filtered', 'Unfiltered');
    grid on;
    
    % Acceleration in Y
    subplot(3,1,2); hold on;
    plot(Xout(2,:), 'b', 'LineWidth', 2);
    plot(measured_data(2,:), '--');
    title('UKF Acceleration Data Y');
    xlabel('Data Points');
    ylabel('Acceleration (m/s^2)');
    legend('Filtered', 'Unfiltered');
    grid on;
    
    % Acceleration in Z
    subplot(3,1,3); hold on;
    plot(Xout(3,:), 'b', 'LineWidth', 2);
    plot(measured_data(3,:), '--');
    title('UKF Acceleration Data Z');
    xlabel('Data Points');
    ylabel('Acceleration (m/s^2)');
    legend('Filtered', 'Unfiltered');
    grid on;
    
    
    figure(2); clf(2); 
    
    % Rotation Rate About X
    subplot(3,1,1); hold on; 
    plot(Xout(4,:), 'b', 'LineWidth', 2);
    plot(measured_data(4,:), '--');
    
    title('UKF Rotation Rate X');
    xlabel('Data Points');
    ylabel('Rotation Rate (dps)');
    legend('Filtered', 'Unfiltered');
    grid on;
    
    % Rotation Rate About Y
    subplot(3,1,2); hold on;
    plot(Xout(5,:), 'b', 'LineWidth', 2);
    plot(measured_data(5,:), '--');
    title('UKF Rotation Rate Y');
    xlabel('Data Points');
    ylabel('Rotation Rate (dps)');
    legend('Filtered', 'Unfiltered');
    grid on;
    
    % Rotation Rate About Z
    subplot(3,1,3); hold on;
    plot(Xout(6,:), 'b', 'LineWidth', 2);
    plot(measured_data(6,:), '--');
    title('UKF Rotation Rate Z');
    xlabel('Data Points');
    ylabel('Rotation Rate (dps)');
    legend('Filtered', 'Unfiltered');
    grid on;
end