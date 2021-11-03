%% ----- Task 1 - ACTIVE SAFETY PROJECT -----
% Version: 2021
% Course: TME 192 Active Safety
%         Chalmers
% Author: Alberto Morando (morando@chalmers.se)
%         Alexander Rasch (arasch@chalmers.se)
%         Marco Dozza (dozza@chalmers.se)
%
% Group: [14
%         Aparna Ram Suresh Saritha Kumari 
%         Elizabeth Swathika Azariah
%         Sundar Murugan Ramaswamy]
%

clear all;
clc;

%% A hypothetical rear-end conflict situation
V = 90*(5/18);       %m/s
g = 9.8;            %m/s^2
a = -5;             %m/s^2
% mu = -a/g;             %no unit
react_time = 1.5;      %s
range = 90;  %m
%% (a) minimum range
min_range = -V^2 / (2*a);     %m

%min range = 62.5m 

%% (b) time-to-collision (TTC)
TTC = min_range/V;

%TTC = 2.5 seconds

%% (c) range with driver's reaction time
react_dist = (V*react_time);  %m
dist_req = react_dist + min_range;   %m

%collision cannot be avoided

%% (d) hypothetical rear-end conflict animation
dist_avail = range - react_dist;  %m
new_TTC = react_time +TTC;  %s
TTC_unreact = (linspace(0,range/V,10))';
speed_unreact = 25 * ones(10,1);
dist_unreact = 90- (speed_unreact.*TTC_unreact);
% Road
road_Width              = 10;       % Road width                    [m]
road_Margin             = 2;        % Road margin                   [m]
% Vehicle
vehicle_Length          = 5;     % Length of the vehicle         [m]
vehicle_Width           = 2;     % Width of the vehicle          [m]
V   = 90/3.6;   % Initial speed of the vehicle  [m/s]
% Parameters
fR      = 30;                       % Frame rate                    [fps]
    
% Braking deceleration
acce = -5; %m/s^2
dist = -V^2/(2*acce); %m
TTCreq = sqrt(-(2*min_range)/acce);
acce_req = -V^2/(2*52.5);  %52.5 is the distance travelled until reaction time
% Simulation
% States
vehicle_speed_2       = transpose(linspace(V,0,30));
vehicle_pos_2    = -vehicle_speed_2.^2/(2*acce_req); 

time_span_1 = (linspace(-2.5,1.5,30))';
time_span_2 = sqrt(-(2*vehicle_pos_2)/acce);
ts = [time_span_1;flip(time_span_2)+time_span_1(30)];

vehicle_position_1    = 90-(V*time_span_1);    %time_span_1 (0 to 1.5s)
vehicle_speed_1       = zeros(30,1) + 25;

vp = [vehicle_position_1;(vehicle_pos_2)];
vs = [vehicle_speed_1;(vehicle_speed_2)];


% Animation
figure
set(gcf,'Position',[50 50 1280 720]) 
v = VideoWriter('braking_dynamics.mp4','MPEG-4');
v.Quality = 100;
v.FrameRate = fR;
open(v);
for i=1:length(ts)
    subplot(2,2,1)
        hold on ; grid on
        set(gca,'xlim',[-2.5 ts(end)],'ylim',[0 180])
        cla 
        plot(ts,vp)
        plot(TTC_unreact,dist_unreact)
        plot([ts(i) ts(i)],[0 180],'k--')
        xline(0, '-c', 'react')
        xline(range/V, '-r', 'Crash')
        xline(react_time, '-m', 'Brake')
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
        txt = ['Travel distance = ' {vp(i)}];
        text(2 ,100 ,txt);
    subplot(2,2,2)
        hold on ; grid on
        set(gca,'xlim',[-2.5 ts(end)],'ylim',[0 1.2*max(vehicle_speed_2)])
        cla 
        plot(ts,vs)
        plot(TTC_unreact,speed_unreact)
        plot([ts(i) ts(i)],[0 1.2*max(vehicle_speed_2)],'k--')
        xline(0, '-c', 'react')
        xline(range/V, '-r', 'Crash')  
        xline(react_time, '-m', 'Brake')
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed')
        txt = ['Driver speed = ' {vs(i)}];
        text(4.5 ,20 ,txt);
    subplot(2,2,3:4)
        hold on ; axis equal
        cla 
        % Position of driver [m]
        vp_1 = [75;75;75];
        vp_2 = [75;75;75];
        vp_3 = [75;75;75];
        vp_4 = [75;75;75];
        vp_5 = [75;75;75];
        vp_6 = [75;75;75];
        vp_7 = [75;75;75];
        vp_8 = vp(22:24);
        vp_9 = vp(25:27);
        vp_10 = vp(28:30);
        vp_11 = vp(31:33);
        vp_12 = vp(34:36);
        vp_13 = vp(37:39);
        vp_14 = vp(40:42);
        vp_15 = vp(43:45);
        vp_16 = vp(46:48);
        vp_17 = vp(49:51);
        vp_18 = vp(52:54);
        vp_19 = vp(55:57);
        vp_20 = zeros(3,1);
        vpi = [vp_1;vp_2;vp_3;vp_4;vp_5;vp_6;vp_7;vp_8;vp_9;vp_10;vp_11;vp_12;vp_13;vp_14;vp_15;vp_16;vp_17;vp_18;vp_19;vp_20];
        vpi = flip(vpi);
        vp_inst = vpi(i);
        road_Length       = 1.5*max(vpi); % Road length [m]
        sideMarkingsX     = [-1.5*vehicle_Length road_Length];
        
        set(gca,'xlim',sideMarkingsX,'ylim',[-road_Width/2-road_Margin +road_Width/2+road_Margin])
        plot(sideMarkingsX,[+road_Width/2 +road_Width/2],'k--') % Left marking
        plot(sideMarkingsX,[-road_Width/2 -road_Width/2],'k--') % Right marking
        % Dimensions
        vehicle_dimension_X1 = [vp_inst vp_inst vp_inst-vehicle_Length vp_inst-vehicle_Length];
        vehicle_dimension_Y1 = [+vehicle_Width/2 -vehicle_Width/2 -vehicle_Width/2 +vehicle_Width/2];
        % Plotting
        fill(vehicle_dimension_X1,vehicle_dimension_Y1,'b')
        
        % Position of the stationary vehicle [m]
        vp_stationary = zeros(60,1) + 90 + vehicle_Length;        
        % Dimensions of the stationary vehicle [m]
        vehicle_dimension_X2 = [vp_stationary vp_stationary vp_stationary-vehicle_Length vp_stationary-vehicle_Length];
        vehicle_dimension_Y2 = [+vehicle_Width/2 -vehicle_Width/2 -vehicle_Width/2 +vehicle_Width/2];
        % Plotting of the stationary vehicle [m]
        fill(vehicle_dimension_X2,vehicle_dimension_Y2,'r')
        
        xlabel('Longitudinal distance [m]')
        ylabel('Lateral distance [m]')
        txt = ['Crash time = ' {new_TTC}];
        text(90 ,-3.5 ,txt);     
        txt_1 = ['Brakes applied'];
        text(70 ,-3.5 ,txt_1);  

    frame = getframe(gcf);
    writeVideo(v,frame);
end 
% Additional frames repeating the last frame
% Parameters
TTC_add      = 3.5;                    % Final time                        [s]
fR_add      = 20;                   % Frame rate                        [fps]
dt_add      = 1/fR;                 % Time resolution                   [s]
time_add    = linspace(0,TTC_add,TTC_add*fR_add); % Time                  [s]
for i=1:length(time_add)
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);




    








