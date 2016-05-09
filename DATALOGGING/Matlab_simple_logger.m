delete(instrfindall); clear; clc; close;

%% Connect to device
MyCOM = serial('COM24', 'Baudrate', 115200); % Wired FTDI
fopen(MyCOM);
fprintf(MyCOM, 'k');
disp('COM open');
%% Collect and plot data from device
i = 1;
test = 0;
instring = fscanf(MyCOM,'%s'); % Throw away first string.

% Sets of data collected before finishing program
len = 1200; %4min = 2400 ,600 = 1 min 
nrdataset = 22;
time = 1:len;

gx = zeros(len,1);
gy = zeros(len,1);
gz = zeros(len,1);

mx = zeros(len,1);
my = zeros(len,1);
mz = zeros(len,1);

ax = zeros(len,1);
ay = zeros(len,1);
az = zeros(len,1);

pitch = zeros(len,1);
roll = zeros(len,1);
heading = zeros(len,1);

depth = zeros(len,1);

th1 = zeros(len,1);
th2 = zeros(len,1);
th3 = zeros(len,1);
th4 = zeros(len,1);
th5 = zeros(len,1);
th6 = zeros(len,1);
th7 = zeros(len,1);
th8 = zeros(len,1);

% figure('units','normalized','outerposition',[0 0 1 1])
 graf = plot(time, gx, time, gx, time, gx);
%  graf = plot(time, gx, 'LineWidth', 2);
 axis([0 len -1000 3500]);
 grid on;
%  legend('th1', 'th2', 'th3');
% legend('mx','my','mz');
% legend('depth');
% legend('ax','ay','az');
legend('depth', 'pitch', 'roll');
%% Collect values from serial
while (i<=len)
    instring = fscanf(MyCOM,'%s');
    M_STRING = strsplit(instring, ',');
    M = zeros(1,nrdataset);
    for teller = 1:nrdataset
        M(teller) = hex2dec(M_STRING(teller));
        if(M(teller)>32768)
            M(teller) = M(teller) - 65536;
        end
    end
    
    % Append data to datasets
    gx(i) = M(1);
    gy(i) = M(2);
    gz(i) = M(3);
    
    mx(i) = M(4);
    my(i) = M(5);
    mz(i) = M(6);
    
    ax(i) = M(7);
    ay(i) = M(8);
    az(i) = M(9);
    
    pitch(i) = M(10);
    roll (i) = M(11);
    heading(i) = M(12);
    
    depth(i) = M(13)*(2^16) + M(14);
    
    th1(i) = M(15);
    th2(i) = M(16);
    th3(i) = M(17);
    th4(i) = M(18);
    th5(i) = M(19);
    th6(i) = M(20);
    th7(i) = M(21);
    th8(i) = M(22);
    
     set(graf(1), 'YData', depth(1:i));
     set(graf(2), 'YData', pitch(1:i));
     set(graf(3), 'YData', roll(1:i));
     
%     % X-AKSE:
     set(graf(1), 'XData', time(1:i));
     set(graf(2), 'XData', time(1:i));
     set(graf(3), 'XData', time(1:i));
    drawnow
    i = i + 1;
end
%% Close connection to device
fprintf(MyCOM, 's');
fclose(MyCOM);
disp('COM closed');
%% Compute min-max values, offset and scaling of sensor axes
% clear, close, clc;
%
% % Load datasets into matlab.
% load('matlab_mxr_dataset.mat');
% load('matlab_myr_dataset.mat');
% load('matlab_mzr_dataset.mat');
%
% % Sort arrays in descending order.
% mxr_sorted = sort(m_x_r_plot);
% myr_sorted = sort(m_y_r_plot);
% mzr_sorted = sort(m_z_r_plot);
%
% % Extract 50 first and 50 last in sorted arrays (top and bottom 5% of
% % measurements).
% mxr_bot = mxr_sorted(1:50);
% mxr_top = mxr_sorted(950:1000);
% myr_bot = myr_sorted(1:50);
% myr_top = myr_sorted(950:1000);
% mzr_bot = mzr_sorted(1:50);
% mzr_top = mzr_sorted(950:1000);
%
% % Find average min-max value for each sensor axis.
% mxr_min = mean(mxr_bot);
% mxr_max = mean(mxr_top);
% myr_min = mean(myr_bot);
% myr_max = mean(myr_top);
% mzr_min = mean(mzr_bot);
% mzr_max = mean(mzr_top);
%
% % Calculate offset for each axis.
% mxr_offset = (mxr_min + mxr_max)/2;
% myr_offset = (myr_min + myr_max)/2;
% mzr_offset = (mzr_min + mzr_max)/2;
%
% % Calculate offset-adjusteded min-max values for each sensor axis.
% mxr_min_adj = mxr_min - mxr_offset;
% mxr_max_adj = mxr_max - mxr_offset;
% myr_min_adj = myr_min - myr_offset;
% myr_max_adj = myr_max - myr_offset;
% mzr_min_adj = mzr_min - mzr_offset;
% mzr_max_adj = mzr_max - mzr_offset;
%
% % Calculate scaling factor so the resulting values lie within [1, -1].
% mxr_scale = (mxr_max_adj - mxr_min_adj)/2;
% myr_scale = (myr_max_adj - myr_min_adj)/2;
% mzr_scale = (mzr_max_adj - mzr_min_adj)/2;
%
% % Plot datasets and calculated min-max values.
% plot(1:1000, m_x_r_plot);
% axis([0 1000 -700 700]);
% hold on;
% grid on;
% plot([1 1000], [mxr_max mxr_max] , '--', 'LineWidth', 1.5);
% plot([1 1000], [mxr_min mxr_min], '--', 'LineWidth', 1.5);
% plot([1 1000], [mxr_offset mxr_offset],  '--', 'LineWidth', 1.5);
% legend('mxr', 'mxr\_max', 'mxr\_min', 'mxr\_offset');
% xlabel('tid [100ms]');
% ylabel('Råverdi fra magnetometer');
%
% figure;
% plot(1:1000, m_y_r_plot);
% axis([0 1000 -700 700]);
% hold on;
% grid on;
% plot([1 1000], [myr_max myr_max] , '--', 'LineWidth', 1.5);
% plot([1 1000], [myr_min myr_min], '--', 'LineWidth', 1.5);
% plot([1 1000], [myr_offset myr_offset],  '--', 'LineWidth', 1.5);
% legend('myr', 'myr\_max', 'myr\_min', 'myr\_offset');
% xlabel('tid [100ms]');
% ylabel('Råverdi fra magnetometer');
%
% figure;
% plot(1:1000, m_z_r_plot);
% axis([0 1000 -700 700]);
% hold on;
% grid on;
% plot([1 1000], [mzr_max mzr_max] , '--', 'LineWidth', 1.5);
% plot([1 1000], [mzr_min mzr_min], '--', 'LineWidth', 1.5);
% plot([1 1000], [mzr_offset mzr_offset],  '--', 'LineWidth', 1.5);
% legend('mzr', 'mzr\_max', 'mzr\_min', 'mzr\_offset');
% xlabel('tid [100ms]');
% ylabel('Råverdi fra magnetometer');