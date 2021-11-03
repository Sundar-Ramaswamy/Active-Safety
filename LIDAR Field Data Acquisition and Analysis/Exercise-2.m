%% ----- Exercise 5 - Field data acquisition and analysis -----
% Version: 2021
% Course: TME 192 Active Safety
%         Chalmers
% Author: Alberto Morando - morando@chalmers.se
%         Christian-Nils Boda - boda@chalmers.se
%         Alexander Rasch - arasch@chalmers.se
%         Pierluigi Olleja - ollejap@chalmers.se
%
% Group: [PLEASE FILL OUT]
%

close all
clear all
clc

%% WARNING
% Before using this script, convert the BAG file into a CSV file with the
% function `bag2csv`. For example `bag2csv('2018-10-24-11-11-25_0.bag')`
% The function will convert all LIDAR data and skip any other data (like
% video data etc.)

%% DATA LOADING
% `importLidarData` Fill a structure containing the recorded data. The structure is as follow:
% +-------------+-------+---------------------------------------------------------------------+
% | Timestamp_s | [s]   | n x 1 vector, where n is the number of timestamps (or frames)       |
% | Angle_rad   | [rad] | 1 x n vector, where n is the number of scanned angles.              |
% |             |       | The range of angles is [-1.658063, 1.658063] radians (-95, +95 deg) |
% | Distance_m  | [m]   | m x n matrix, where m is the number of timestamps (or frames),      |
% |             |       | and n is the number the scanned angle.                              |
% |             |       | Each value represents the distance at a given time stamp and angle  |
% +-------------+-------+---------------------------------------------------------------------+

filename_csv = '2021-10-04-15-00-54_0.csv'; % <--- Change this one with your own file!

data = importLidarData( filename_csv );

% Sampling settings
sample_period_s = median(diff(data.Timestamp_s));

%% SELECT THE EVENT OF INTEREST
% Find the relevant events for your dataset using the function playLidar().
% With `playLidar` determine what is the optimal FOV to detect the
% obstacles, and write down the start and stop time index (frame) for each event into `index_of_events`.
% The matrix `index_of_events` should follow the following template:
%
% +-------+------+
% | start | stop |
% +-------+------+
% |     1 |  123 |
% |   156 |  236 |
% |   ... |  ... |
% +-------+------+

% Uncomment the line to use `playLidar`
% playLidar('2021-10-04-15-00-54_0.csv');

% Store your event indices in a matrix like the following (example)
index_of_events = [ 
    3985,      3995;
    4318,      4327;
    5825,      5838;
    8100,      8145;% <--- Change this with respect to your data!
];

% Change the FOV limits to what you found that gave the best results
FOV_limits_deg = 15; % <--- Change this with respect to your data!

FOV_limits_rad =  deg2rad([-FOV_limits_deg/2, FOV_limits_deg/2]);

% Extract the index for the columns within the selected field of view (FOV)
FOV_span_idx = [find(data.Angle_rad >= FOV_limits_rad(1), 1, 'first') : find(data.Angle_rad <= FOV_limits_rad(end), 1, 'last')];

% Change the FOV limits to what you found that gave the best results
Full_limits_deg = 190; % <--- Change this with respect to your data!

Full_limits_rad =  deg2rad([-Full_limits_deg/2, Full_limits_deg/2]);

% Extract the index for the columns within the selected field of view (FOV)
Full_span_idx = [find(data.Angle_rad >= Full_limits_rad(1), 1, 'first') : find(data.Angle_rad <= Full_limits_rad(end), 1, 'last')];

%% PLOT A SNAPSHOT OF THE LIDAR MEASUREMENT
% Select a _single_ frame from one of your events and plot the LIDAR
% measurement
frameNumber = 4327;  % <--- Change this with respect to your data!

figure('name', 'Example of FOV');
hold on;

% Plot the LIDAR origin
plot(0, 0, 'r^', 'markersize', 10, 'MarkerFaceColor', 'r');
hold on;

% Plot the distance measurements in the full field of view
Dist_frame = data.Distance_m(frameNumber,Full_span_idx);
[y,x] = pol2cart(data.Angle_rad(Full_span_idx),Dist_frame);
plot(x,y,'-k');
hold on;

% % Extract the data in the chosen FOV
Dist_frame = data.Distance_m(frameNumber,FOV_span_idx);
[y,x] = pol2cart(data.Angle_rad(FOV_span_idx),Dist_frame);
plot(x, y, '-k', 'linewidth', 2);
hold on;

% find the FOV index for the closest point in the field of view
for index=FOV_span_idx(1):1:(FOV_span_idx(1)+size(FOV_span_idx,2)-1)
    if data.Distance_m(frameNumber,index) == min(data.Distance_m(frameNumber,FOV_span_idx))
        closest_index=index;
    end
end

% find the x and y co-ordinates for the closest point in the field of view
% and plot the closest point
x_closest = data.Distance_m(frameNumber,closest_index)*sin(data.Angle_rad(closest_index));
y_closest = data.Distance_m(frameNumber,closest_index)*cos(data.Angle_rad(closest_index));
plot(x_closest, y_closest, 'xb', 'linewidth', 1.2, 'markersize', 8);


% Plot the boundaries of the selected field of view
plot([0,-15], [0,100], ':r', 'linewidth', 1.5) % left limit
plot([0,15], [0,100], ':r', 'linewidth', 1.5) % right limit

% For a better visualization you should limit the axis in the figure
set(gca, 'xlim', [-15,15], 'ylim', [0,25])
axis equal

xlabel('Distance [m]'); 
ylabel('Distance [m]');

legend('LIDAR', 'Data_{all}', 'Data_{FOV}', 'Closest point', 'Boundaries of ','the field of view','Location','northwest')

%% PLOT DISTANCE OF TARGET ACROSS TIME
figure;
% x=[];
% for i=1:1:size(data.Distance_m,1)
%     x(end+1)=min(data.Distance_m(i,FOV_span_idx));
% end
for j=8045:1:8080
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-402.25;
    plot(time,dist,'r.');
    hold on
end
hold on
for k=5060:1:5111
    dist=min(data.Distance_m(k,:));
    time=data.Timestamp_s(k)-253;
    plot(time,dist,'g.');
    hold on;
end


for j=8104:1:8145
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-405.2;
    plot(time,dist,'b.');
    hold on;
end
for j=8276:1:8323
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-413.75;
    plot(time,dist,'m.');
    hold on;
end

for j=5308:1:5350
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-265.35;
    plot(time,dist,'c.');
    hold on;
end
    
xlim([0 4]);

% hold on
% plot(data.Distance_m(4320:4327,FOV_span_idx));
% hold on
% plot(data.Distance_m(4327,760:770));
% hold on
% plot(data.Distance_m(5760,397:407));
% hold on
% plot(data.Distance_m(5767,617:627));
% hold on
% plot(data.Distance_m(8140,1343:1353));
% hold on
% plot(data.Distance_m(8145,1299:1309));
% hold on
xlabel('Time [s]'); ylabel('Distance [m]')

%% PLOT RELATIVE SPEED TO TARGET ACROSS TIME
figure;
hold on
for j=8045:1:8080
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    prev_time=data.Timestamp_s(j-1)-402.2;
    time=data.Timestamp_s(j)-402.25;
    speed=(dist-prev_dist)/(time);
    plot(time,speed,'r.');
    hold on
end
hold on
for k=5060:1:5111
    prev_dist=min(data.Distance_m(k-1,:));
    dist=min(data.Distance_m(k,:));
    time=data.Timestamp_s(k)-253;
    speed=(dist-prev_dist)/(time);
    plot(time,speed,'b.');
    hold on;
end


for j=8104:1:8145    
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-405.2;
    speed=(dist-prev_dist)/(time);
    plot(time,speed,'r.');
    hold on;
end
for j=8276:1:8323
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-413.75;
    speed=(dist-prev_dist)/(time);
    plot(time,speed,'m.');
    hold on;
end

for j=5308:1:5350
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-265.35;
    speed=(dist-prev_dist)/(time);
    plot(time,speed,'c.');
    hold on;
end
    
xlim([0 4]);

xlabel('Time [s]'); ylabel('Relative speed [m/s]')

%% PLOT TIME TO COLLISION TO TARGET ACROSS TIME
figure;
hold on

for j=8045:1:8080
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    prev_time=data.Timestamp_s(j-1)-402.2;
    time=data.Timestamp_s(j)-402.25;
    speed=(dist-prev_dist)/(time);
    ttc=dist/speed;
    plot(time,ttc,'r.');
    hold on
end
hold on
for k=5060:1:5111
    prev_dist=min(data.Distance_m(k-1,:));
    dist=min(data.Distance_m(k,:));
    time=data.Timestamp_s(k)-253;
    speed=(dist-prev_dist)/(time);
    ttc=dist/speed;
    plot(time,ttc,'b.');
    hold on;
end


for j=8104:1:8145    
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-405.2;
    speed=(dist-prev_dist)/(time);
    ttc=dist/speed;
    plot(time,ttc,'r.');
    hold on;
end
for j=8276:1:8323
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-413.75;
    speed=(dist-prev_dist)/(time);
    ttc=dist/speed;
    plot(time,ttc,'m.');
    hold on;
end

for j=5308:1:5350
    prev_dist=min(data.Distance_m(j-1,:));
    dist=min(data.Distance_m(j,:));
    time=data.Timestamp_s(j)-265.35;
    speed=(dist-prev_dist)/(time);
    ttc=dist/speed;
    plot(time,ttc,'c.');
    hold on;
end

xlabel('Time [s]'); ylabel('TTC [s]')

%% --- Question ---
% What safety measure would you use to design a warning that alert the user
% that is about to collide with an obstacle? You  may want to use one of the safety measure 
% you computed in this script of find a more effective one.
% What value of such measure would you use to trigger a warning?
% (Write your answer and enclose it in a comment)


