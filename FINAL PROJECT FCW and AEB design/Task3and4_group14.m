%% ----- Task 3 and 4 - ACTIVE SAFETY PROJECT -----
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

load('TableTask2.mat')
load('Safety_Metrics.mat')
figure(1)
plot(T.speed_at_BO(1:12), T.TTC_at_BO(1:12),'ro')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 1')

figure(2)
plot(T.speed_at_BO(13:18), T.TTC_at_BO(13:18),'bo')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 2')

figure(3)
plot(T.speed_at_BO(19:25), T.TTC_at_BO(19:25),'mo')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 4')

figure(4)
plot(T.speed_at_BO(26:34), T.TTC_at_BO(26:34),'go')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 5')

figure(5)
plot(T.speed_at_BO(35:42), T.TTC_at_BO(35:42),'mo')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 6')

figure(6)
plot(T.speed_at_BO(43:53), T.TTC_at_BO(43:53),'ko')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 7')

figure(7)
plot(T.speed_at_BO(54:63), T.TTC_at_BO(54:63),'ro')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 8')

figure(8)
plot(T.speed_at_BO(64:73), T.TTC_at_BO(64:73),'bo')
xlabel('Speed at Brake Onset(m/s)')
ylabel('TTC at brake onset(s)')
title('participant 9')

figure
histogram(T.TTC_at_BO,12)
title('Histogram representation of TTC')
percentile_5th = prctile(T.TTC_at_BO,5); %5th percentile
percentile_95th = prctile(T.TTC_at_BO,95); %95th percentile

% I would design a system for 95th percentile ttc
figure
histogram(Safety_Metrics.Mean_Acceleration,12)
title('Histogram representation of mean acceleration')
figure
histogram(Safety_Metrics.Min_Acceleration,12)
title('Histogram representation of min acceleration')

% for instance if a driver is aggressive it is more possible to decelerate hard and jerkily (large
% deceleration value and short duration)

%% Task 4
% Numerical thresholds for the conservative system(FCW cons)
average_speed_at_BO = mean(T.speed_at_BO,'all'); %average speed at brake onset
TTC_range = linspace(percentile_95th,percentile_5th,30);
percentile_90th = prctile(T.TTC_at_BO,90); %90th percentile
react_time = 1.5; %s
TTC_task1 = 2.5; %s
TTC_conservative = react_time + TTC_task1 + 2.5;
V = 25;  %m/s
dist_cons = V * TTC_conservative;
min_range = 90; %m
a_cons = V^2/(2*dist_cons); %m/s^2

% Numerical thresholds for the aggressive system(FCW aggr)
percentile_50th = prctile(T.TTC_at_BO,50); %50th percentile
TTC_aggressive = react_time + TTC_task1 + 0.5;   
dist_aggressive = V * TTC_aggressive;
a_aggr = V^2/(2*dist_aggressive); %m/s^2

%% Pseudocode for warning systems
% while Velocity > 0 % (DRIVING)
%     Monitor TTC values %threat assessment
%     if TTC value == TTC_conservative %decision making
%         issue warning
%     end
% end
% while Velocity > 0 % (DRIVING)
%     Monitor TTC values %threat assessment
%     if TTC value == TTC_aggressive %decision making
%         issue warning
%     end
% end
%%   
max_brake_ability = -10; %m/s^2  % min acceleration from table  also safety_metrics lesser 
% than 5th percentile of min acc
dist_AEB_1 = -V^2/(2*max_brake_ability); %m
TTCreq_AEB = sqrt(-(2*min_range)/max_brake_ability); %Time when vehicle speed reaches zero
TTC_avail_AEB_1 = dist_AEB_1/V; %TTC required before braking to avoid crash
%% Pseudocode for AEB acc (system 1)
% while Velocity > 0 % (DRIVING)
%     Monitor TTC values %threat assessment
%     if TTC value == TTC_avail_AEB_1 %decision making
%         brake intervention %decision making
%     end
% end

%% 
TTC_AEB_2 = 1.0220; %s % 10th percentile of TTC at BO
dist_AEB_2 = V * TTC_AEB_2;
acc_AEB_2 = -V^2/(2*dist_AEB_2);
TTCreq_AEB_2 = sqrt(-(2*dist_AEB_2)/acc_AEB_2); %Time when vehicle speed reaches zero
%The advantage of the above method is it comes to stop before 5 meters from the vehicle.

%A simple AEB system may brake a car unexpectedly in urban situations while
%driving in traffic
%AEB should not intervene when the speed is less than specific value to encounter such a case
%% Pseudocode for AEB TTC (system 2)
% while Velocity > 0 % (DRIVING)
%     Monitor TTC values %threat assessment
%     if TTC value == TTC_AEB_2 %decision making
%         brake intervention %decision making
%     end
% end

%%
TTC_avail = (linspace(0, min_range/V, 30))';  %s
vehicle_speed      = 25*ones(30); %no reaction
vehicle_speed = vehicle_speed(:,1); %no reaction
vehicle_position    = 90-(V.*TTC_avail);  %no reaction

TTC_conservative_1 = TTC_avail(30)-TTC_conservative;
TTC_aggressive_1 = TTC_avail(30)-TTC_aggressive;

%% for AEB system 1. Here acceleration(maximum braking ability) is the threat assessment factor..
veh_spd_2 = (linspace(V,0,15))'; 
veh_pos_2 = -veh_spd_2.^2/(2*max_brake_ability);

time_span_1 = (linspace(0,TTC_avail(30)-TTC_avail_AEB_1,15))';
time_span_2 = sqrt(-(2*veh_pos_2)/max_brake_ability);
ts_12 = [time_span_1;flip(time_span_2)+time_span_1(15)];

veh_spd_1 = 25*ones(15); %no reaction
veh_spd_1 = veh_spd_1(:,1); %no reaction
veh_pos_1 = 90-(veh_spd_1.*time_span_1);  %no reaction

vehicle_position_1 = [veh_pos_1;(veh_pos_2)];
vehicle_speed_1 = [veh_spd_1;veh_spd_2];

%% for AEB system 2. Here TTC is the threat assessment factor.
veh_spd_4 = (linspace(V,0,15))'; 
veh_pos_4 = -veh_spd_2.^2/(2*acc_AEB_2);

time_span_3 = (linspace(0,TTC_avail(30)-TTC_AEB_2,15))';
time_span_4 = sqrt(-(2*veh_pos_4)/acc_AEB_2);
ts_34 = [time_span_3;flip(time_span_4)+time_span_3(15)];

veh_spd_3 = 25*ones(15); %no reaction
veh_spd_3 = veh_spd_3(:,1); %no reaction
veh_pos_3 = 90-(veh_spd_3.*time_span_3); %no reaction

vehicle_position_2 = [veh_pos_3;(veh_pos_4)];
vehicle_speed_2 = [veh_spd_3;veh_spd_4];
%% task 1 graph plot
acce = -5; %m/s^2
dist = -V^2/(2*acce); %m
TTCreq = sqrt(-(2*min_range)/acce);
acce_req = -V^2/(2*52.5);  %52.5 is the distance travelled until reaction time
% Simulation
% States
vehicle_speed_5       = transpose(linspace(V,0,30));
vehicle_pos_5    = -vehicle_speed_5.^2/(2*acce_req); 

time_span_5 = (linspace(-3.5,1.5,30))';
time_span_6 = sqrt(-(2*vehicle_pos_5)/acce);
ts_56 = [time_span_5;flip(time_span_6)+time_span_5(30)];

vehicle_position_3    = 90-(V*time_span_5);    %time_span_1 (0 to 1.5s)
vehicle_speed_3       = zeros(30,1) + 25;

vp = [vehicle_position_3;(vehicle_pos_5)];
vs = [vehicle_speed_3;(vehicle_speed_5)];
%% final plot
for i=1:length(TTC_avail)
    subplot(2,1,1)
        hold on ; grid on
        set(gca,'xlim',[-3.5 1.8*TTC_avail(end)],'ylim',[0 2*max(vehicle_position)])
        cla 
        plot(TTC_avail,vehicle_position, 'g','LineWidth',1)
        plot(ts_12,vehicle_position_1, 'r','LineWidth',1)
        plot(ts_34,vehicle_position_2, 'k','LineWidth',1)
        plot(ts_56, vp, 'b','LineWidth',1)
        xline(TTC_avail(30), '--r', 'Crash')
        xline(0, '--r', 'react')
        xline(1.5, '--r', 'Brake')
        xline(TTC_conservative_1, '--m', 'FCW cons')
        xline(TTC_aggressive_1, '--m', 'FCW aggr')
        xline(TTC_avail(30)-TTC_avail_AEB_1, '--c', 'AEB dec')
        xline(TTC_avail(30)-TTC_AEB_2, '--c', 'AEB TTC')
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')

    subplot(2,1,2)
        hold on ; grid on
        set(gca,'xlim',[-3.5 1.8*TTC_avail(end)],'ylim',[0 1.2*max(vehicle_speed)])
        cla 
        plot(TTC_avail,vehicle_speed, 'g','LineWidth',1)
        plot(ts_12,vehicle_speed_1, 'r','LineWidth',1)
        plot(ts_34,vehicle_speed_2, 'k','LineWidth',1)
        plot(ts_56, vs, 'b','LineWidth',1)
        xline(TTC_avail(30), '--r', 'Crash')
        xline(0, '--r', 'react')
        xline(1.5, '--r', 'Brake')
        xline(TTC_conservative_1, '--m', 'FCW cons')
        xline(TTC_aggressive_1, '--m', 'FCW aggr')
        xline(TTC_avail(30)-TTC_avail_AEB_1, '--c', 'AEB dec')
        xline(TTC_avail(30)-TTC_AEB_2, '--c', 'AEB TTC')
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed')

end









