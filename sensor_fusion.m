%% Import dataset
clear;
clc;
close all;
%load('P_I_D_reg_depth_15N.mat');
%load('testrun1.mat');
load('plattform1_roll.mat');

%% Filter fusion gyro and accelerometer
% Initialize vectors
dataset_length = length(ax);
delta_pitch_acc = zeros(1, dataset_length);
delta_roll_acc = zeros(1, dataset_length);
delta_pitch_gyro = zeros(1, dataset_length);
delta_roll_gyro = zeros(1, dataset_length);
delta_yaw_gyro = zeros(1, dataset_length);
est_delta_pitch = zeros(1, dataset_length);
est_delta_roll = zeros(1, dataset_length);
est_pitch = zeros(1, dataset_length);
est_roll = zeros(1, dataset_length);
weight_acc = zeros(1, dataset_length);
weight_acc_roll = zeros(1, dataset_length);
weight_acc_pitch = zeros(1, dataset_length);
weight_gyro_pitch = zeros(1, dataset_length);
weight_gyro_roll = zeros(1, dataset_length);
weight_gyro_yaw = zeros(1, dataset_length);

% Initial condition
pitch_k_1 = 0;
roll_k_1 = 0;
timestep = 0.1; % 10 Hz sampling frequency.

for i=1:dataset_length
    
   % ----------------------------------------------------------------------
   % ACCELEROMETER
   % ----------------------------------------------------------------------
   ax_k = ax(i)/1000;
   ay_k = ay(i)/1000;
   az_k = az(i)/1000;
   
   % Normalize measurements.
   abs_k = sqrt(ax_k^2 + ay_k^2 + az_k^2);
   ax_k = ax_k/abs_k;
   ay_k = ay_k/abs_k;
   az_k = az_k/abs_k;
   
   % Calculate pitch, roll
   pitch_k = atan2(ax_k, sqrt(ay_k^2 + az_k^2));
   roll_k = atan2(ay_k, az_k);
   
   % Rad -> deg
   pitch_k = pitch_k * 180 / pi;
   roll_k = roll_k * 180 / pi;
   
   % Calculate delta_pitch, delta_roll
   if(i>1)      
      delta_pitch_acc(i) = pitch_k - est_pitch(i-1);
      delta_roll_acc(i) = roll_k - est_roll(i-1);
   else 
      delta_pitch_acc(i) = pitch_k;
      delta_roll_acc(i) = roll_k;
   end
   
   
   % Calculate accelerometer weight
   abs_diff = abs(abs_k - 1);
   
   % Thruster compensation
   th_abs_k = sqrt(th1(i)^2 + th2(i)^2 + th3(i)^2 + th4(i)^2 + th5(i)^2 + th6(i)^2 ...
    + th7(i)^2 + th8(i)^2);
   th_abs_k = th_abs_k/200;
   
   weight_acc(i) = 1/(abs_diff + 1);
   
   if(th_abs_k < 0.25)
       %weight_acc(i) = weight_acc(i) - 3*th_abs_k;
   else 
       %weight_acc(i) = weight_acc(i) - 0.75;
   end
   delta_pitch_k = pitch_k - pitch_k_1;
   delta_roll_k = roll_k - roll_k_1;
   pitch_k_1 = pitch_k;
   roll_k_1 = roll_k;
   
   if (delta_pitch_k > 1)
       weight_acc_pitch(i) = abs(weight_acc(i)/(delta_pitch_k));
   else
       weight_acc_pitch(i) = weight_acc(i);
   end
   if (delta_roll_k > 1)
       weight_acc_roll(i) = abs(weight_acc(i)/(delta_roll_k));
   else
       weight_acc_roll(i) = weight_acc(i);
   end
   
   
   % ----------------------------------------------------------------------
   % GYROSCOPE
   % ----------------------------------------------------------------------
   % ROV
%    gx_k = gx(i)/100;
%    gy_k = gy(i)/100;
%    gz_k = gz(i)/100;
   
   % Platform:
   gx_k = gx(i)/0.30517;
   gy_k = gy(i)/0.30517;
   gz_k = gz(i)/0.30517;
   
   % Calculate delta_pitch, delta_roll, delta_yaw
   delta_pitch_gyro(i) = timestep*gy_k;
   delta_roll_gyro(i) = timestep*gx_k;
   delta_yaw_gyro(i) = timestep*gz_k;
   
   % Calculate gyroscope weights
   weight_gyro_pitch(i) = abs(gy_k/5);
   weight_gyro_roll(i) = abs(gx_k/5);
   weight_gyro_yaw(i) = abs(gz_k/5);
   
   weight_gyro_pitch(i) = weight_gyro_pitch(i)*2;
   weight_gyro_roll(i) = weight_gyro_roll(i)*2;
   
   % ----------------------------------------------------------------------
   % FUSION
   % ----------------------------------------------------------------------
   % Combine gyroscope and accelerometer estimates:
   est_delta_pitch(i) = delta_pitch_acc(i) * weight_acc_pitch(i) + ...
                            delta_pitch_gyro(i) * weight_gyro_pitch(i);
   est_delta_roll(i) = delta_roll_acc(i) * weight_acc_roll(i) + ...
                        delta_roll_gyro(i) * weight_gyro_roll(i);
   % Divide by sum of weights:
   est_delta_pitch(i) = est_delta_pitch(i)/(weight_acc_pitch(i) + weight_gyro_pitch(i));
   est_delta_roll(i) = est_delta_roll(i)/(weight_acc_roll(i) + weight_gyro_roll(i));
   
   % Calculate new estimate:
   if(i>1)
        est_pitch(i) = est_pitch(i-1) + est_delta_pitch(i);
        est_roll(i) = est_roll(i-1) + est_delta_roll(i);
   else
       est_pitch(i) = est_delta_pitch(i);
       est_roll(i) = est_delta_roll(i);
   end;
end

%% Plot before and after gyro/accelerometer fusion
% 1/10 degrees -> degrees
pitch_acc_plot = (pitch./10)';
roll_acc_plot = (roll./10)';

pitch_fus_plot = est_pitch;
roll_fus_plot = est_roll;
time = 0:timestep:(timestep*dataset_length - timestep);

subplot(3,1,1);
plot(time, roll_acc_plot);hold on; plot(time, roll_fus_plot,'LineWidth', 1.5);
legend('Accelerometer pitch' ,'Fusion pitch');
xlabel('Tid [x100 ms]'); ylabel('Vinkel [grader]');

subplot(3,1,2);
% Roll
% plot(time, roll_acc_plot); hold on; plot(time, roll_fus_plot, 'LineWidth', 1.5);
% legend('Accelerometer roll' ,'Fusion roll');
% xlabel('Tid [x100 ms]'); ylabel('Vinkel [grader]');

% Weights
plot(time, weight_acc_roll, time, weight_gyro_roll, 'LineWidth', 1);
legend('Weight acc pitch', 'Weight gyro pitch'); xlabel('Tid [x100 ms]');

subplot(3,1,3);
% Thrusters
th_abs = sqrt(th1.^2 + th2.^2 + th3.^2 + th4.^2 + th5.^2 + th6.^2 ...
    + th7.^2 + th8.^2);
th_abs = th_abs./200;
plot(time, th_abs, 'LineWidth', 1.5);
legend('Absolutt thrusterp�drag');
xlabel('Tid [x100 ms]');

foer = std(pitch(200:1200))/10;
etter = std(est_pitch(200:1200));
fprintf('Standardavvik f�r var: %.3f \nStandardavvik etter var: %.3f \nDifferansen var: %.3f%%', ...
    foer, etter, (foer-etter)*100/foer);
