%% ----- Exercise 2 - Decode CAN data -----
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

clear all
close all
clc

%% ### Auxiliary functions
% Function to translate hex to binary
hex2bin = @(data) dec2bin(hex2dec(data), 8);

%% ### Import the data
% There are multiple ways to import data from a txt file. One way is to use 'readtable' (read the doc for more
% information). Note that 'format' is a list of `%x` where `x` represent
% the different data types (e.g. `s` for string, `d` for decimal). For the
% exercise, we already defined the format for you.
FORMAT_SPEC = '%f%d%s%s%c%d%s%s%s%s%s%s%s%s';
load('CANdata.mat')

% Add variable names. This would be helpful for analysing the data
CANdata.Properties.VariableNames{1} = 'Timestamp';
CANdata.Properties.VariableNames{3} = 'Identifier';

%% ### Decode the signals
% Select the frames based on the identifier. You could use `strcmpi` on
% the `Identifier` column

%% * Find vehicle speed signal to decode *
count = 1;
for i=1:2839
    s1 = CANdata.Identifier(i);
    s2 = '401';
    comp(i) = strcmpi(s1,s2);
    if comp(i) == true
        speed_index(count,:) = CANdata(i,[1 7 8 9 10 11 12 13 14]);
        count = count+1;
    end
end

% Extract the bytes of interest
speed_hex = [speed_index.Var14];

% Convert hex -> bin 
speed_bin = (hex2bin(speed_hex));
speed_b = cellstr(speed_bin);

% convert bin -> dec (use 'bin2dec'). Apply the 'factor' and `offset`
speed = bin2dec(speed_b)*1+0;

%% * Find the accelerator pedal position signal to decode *
count_1=1;
for j=1:2839
    s3 = CANdata.Identifier(j);
    s4 = '1CC';
    comp_1(j) = strcmpi(s3,s4);
    if comp_1(j) == true
        accelerator_index(count_1,:) = CANdata(j,[1 7 8 9 10 11 12 13 14]);
        count_1 = count_1+1;
    end
end

% Extract the bytes of interest
accelerator_hex = strcat(hex2bin(accelerator_index.Var11),hex2bin(accelerator_index.Var12));

% Convert hex -> bin 
a_sliced = accelerator_hex(:,7:16);
accelerator_bin = cellstr(a_sliced);
%accelerator_bin = bin2dec(a_bin);

% convert bin -> dec (use 'bin2dec'). Apply the 'factor' and `offset`
accelerator = bin2dec(accelerator_bin)*0.098+0;

%% Plot the signals with respect to time. Include labels and legend
speed_time = speed_index.Timestamp;
accelerator_time = accelerator_index.Timestamp;

figure
plot(speed_time,speed,'b',accelerator_time,accelerator,'r');
xlabel('Time (s)')
legend('Vehicle speed','Accelerator pedal position')



%% Downsample the signals
% Remove the delay between the two signals. That is, make sure both signals
% starts from zero.
speed_time_from_0 = speed_time;
accelerator_time_from_0 = accelerator_time - 0.0002;%accelerator_index(1,1);

% Create a master clock, at 1Hz, common to both signals
sync_time = linspace(0,39,40);

% Downsample by interpolation (use 'interp1')
speed_sync = interp1(speed_time_from_0, speed, sync_time);
accelerator_sync = interp1(accelerator_time_from_0, accelerator, sync_time);

%% Plot the signals with respect to time. Include labels and legend. Limit the axis.
figure
plot(speed_time,speed,'b--',sync_time,speed_sync,'b-o');
hold on
plot(accelerator_time,accelerator,'r--',sync_time,accelerator_sync,'r-o');
xlabel('Time (s)')
legend('Vehicle speed','Vehicle speed sync','Accelerator pedal position','Accelerator pedal position sync')
hold off
xlim([28 36])
ylim([15 30])

%% Issue a warning when the driver is about to exceed the speed limit
% Be careful about the units!
SPEED_LIMIT = 50;       % [km/h]
TOLERANCE = 10;         % [km/h]
REACTION_TIME = 1.0;    % [s]

v_k = speed_sync./3.6;
a_k_1 = diff(v_k);
a_k = [0 a_k_1(1:end)];
V_pred = v_k+(a_k*REACTION_TIME);
% Find time when predicted speed is greater than speed limit + tolerance
result = find(V_pred >= 16.66); 
t = result(1);   
warning_time = sync_time(t);

disp(['Warning activated at time ' num2str(warning_time) ' s.']);

return