%% ----- Exercise 6 - Human factors of automated driving -----
% Version: 2021
% Course: TME 192 Active Safety
%         Chalmers
% Author: Alberto Morando - morando@chalmers.se
% Contact: Alexander Rasch - arasch@chalmers.se
%          Pierluigi Olleja - ollejap@chalmers.se
%
% Group 14 
% Group Members : Aparna Ram Suresh Saritha Kumari 
%                 Elizabeth Swathika Azariah
%                 Sundar Murugan Ramaswamy
%

clear all
close all
clc

%% INITIALIZATION (do not modify)
% Import libraries
currentFolder = strsplit(pwd, '\');
parentFolder = fullfile(currentFolder{1:end-1});
addpath(genpath(parentFolder));

% Set the folder containg the data
SOURCE.experimentDataFolder = fullfile(parentFolder, 'data');
SOURCE.dataParticipantsFolder = fullfile(SOURCE.experimentDataFolder, 'data_participants');

% Set the event type:  A -  Rear end scenario
SOURCE.EventType = {'A'};

% Create the list of the participants
filter = '0*';
ParticipantList = dir(fullfile(SOURCE.dataParticipantsFolder, filter));
ParticipantList = {ParticipantList.name};

%%  EXTRACT DATA FROM THE SIMULATOR LOGS (do not modify)
% The structure 'data' contains the event data from the
% simulator logs for each participant.

data = []; 

for iParticipant = 1 : length(ParticipantList)
    
    % Load simulator data (timeseries of speed, acceleration, ...)
    toBeLoaded_SILAB = fullfile(...
        SOURCE.dataParticipantsFolder,...
        ParticipantList{iParticipant}, ...
        sprintf('SILABdata_%s.mat', ParticipantList{iParticipant}));
    
    LOADED = load(toBeLoaded_SILAB);
    
    % Extract the event timeseries from the simulator logs
    [ dataParticipant_ ] = LOAD_simulatorData( LOADED, SOURCE.EventType{1});
    dataParticipant_.DriverID = str2num(ParticipantList{iParticipant});
    
    % Extract the annotations
    dataParticipant_ = LOAD_annotations( SOURCE, ParticipantList{iParticipant}, SOURCE.EventType{1}, dataParticipant_);
    
    % Append the single data participants to the structure
    data = vertcat(data, dataParticipant_);
end

% Display your annotations (do not modify)
PLOT_annotations(SOURCE, ParticipantList, data)

%%  ==== YOUR CODE HERE ====
% The array of structures "data" contains the data for each participant, useful for solving the exercise. It
% contains measure collected from the vehicle and other human factors measures from your
% annotations. Explore the variables included in 'data'. Note that 'EGO' marks measures of subject's vehicle, 'POV' the measure of the front
% vehicle --Principal other vehicle--

%% COMPUTE SAFETY INDICATORS FOR EACH PARTICIPANT:
% Time to collision (TTC)
% Inverse Time to collision (iTTC = 1/TTC)
% Time headway (THW)
% 
% Note: Sometimes it is better to use the inverse of TTC to avoid getting infinite values (and
% instead we get zeros). This metric is also known as "urgency". In fact, the value of inverse TTC
% increases the closer we get to a collision. Make sure to plot TTC, iTTC, and
% THW as positive values. To compute the metrics you may need the range and range_rate between the EGO and POV

for iParticipant = 1 : length(ParticipantList)
    data(iParticipant).TTC = data(iParticipant).EGO_POV_fender_to_fender_range./data(iParticipant).EGO_POV_fender_to_fender_rangeRate;
    data(iParticipant).iTTC = (1./data(iParticipant).TTC);
    data(iParticipant).THW = data(iParticipant).EGO_POV_longitudinal_distance_CoG ./ data(iParticipant).EGO_speed;
end

%% PLOT VEHICLE MEASURES - LONGITUDINAL
% Identify the correct variable in 'data' and plot it
figure('name', 'Vehicle data - Longitudinal', 'position', [70, 110, 600, 650])

for iParticipant= 1 : length(ParticipantList)
    
    % Plot the speed of the ego vehicle relative to the time of the warning onset (Time_relative_to_warning_s)
    subplot(4, 1, 1)
    hold all
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).EGO_speed)
    ylabel('Speed [m/s]')
    
    % Plot the longitudinal acceleration of the ego vehicle relative to the time of the warning onset 
    subplot(4, 1, 2)
    hold all 
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).EGO_longitudinal_acceleration)
    ylabel('Long. acc. [m/s^2]')

    % Plot the inverse TTC relative to the time of the warning onset 
    subplot(4, 1, 3)
    hold all   
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).iTTC)
    ylabel('TTC^{-1} [s^{-1}]')

    % Plot the THW relative to the time of the warning onset
    subplot(4, 1, 4)
    hold all
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).THW)
    ylabel('THW [s]')

    xlabel('Time relative to warning onset [s]')
       
end

legend(ParticipantList)

% Fix visualization
axList = findobj(gcf, 'type', 'axes');
set(axList,...
    'fontsize', 8, ...
    'LabelFontSizeMultiplier', 1, ...
    'xlim', data(iParticipant).Time_relative_to_warning_s([1 end]))

%% PLOT VEHICLE MEASURES - LATERAL
% Indentify the correct variable in 'data' and plot it
figure('name', 'Vehicle data - Lateral', 'position', [70, 110, 600, 650])

for iParticipant = 1 : length(ParticipantList)

    % Plot the lateral acceleration of the ego vehicle relative to the time of the warning onset (Time_relative_to_warning_s)
    subplot(4, 1, 1)
    hold all 
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).EGO_lateral_acceleration)
    ylabel('Lat. acc. [m/s^2]')
    
    % Plot the steering angle of the ego vehicle relative to the time of the warning onset 
    subplot(4, 1, 2)
    hold on
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).EGO_steerAngle)
    ylabel('Steer angle [rad]')

    % Plot the steering angle rate of the ego vehicle relative to the time of the warning onset       
    subplot(4, 1, 3)
    hold on
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).EGO_steerAngleRate)
    ylabel('Steer angle rate [rad/s]')

    % Plot the steering angle jerk of the ego vehicle relative to the time of the warning onset
    subplot(4, 1, 4)
    hold on
    plot(data(iParticipant).Time_relative_to_warning_s, data(iParticipant).EGO_steerAngleJerk)
    ylabel('Steer angle jerk [rad/s^2]')

    xlabel('Time relative to warning onset [s]') 
    
end

legend(ParticipantList)

% Fix visualization
axList = findobj(gcf, 'type', 'axes');
set(axList,...
    'fontsize', 8, ...
    'LabelFontSizeMultiplier', 1, ...
    'xlim', data(iParticipant).Time_relative_to_warning_s([1 end]))

%% STUDY THE RESPONSE PROCESS
% Compute the reaction time to brake hovering and hands on wheel of the drivers in seconds. 
% Here you should combine the variables "Time_relative_to_warning_s", "first_frame_hovering_over_brake_pedal", and
% 'first_frame_hands_on_wheel'. The variables with prefix 'first_frame_' have been derived from your annotation.

for iParticipant = 1 : length(ParticipantList)
    
    data(iParticipant).hovering_over_brake_pedal_time = data(iParticipant).Time_relative_to_warning_s(data(iParticipant).first_frame_hovering_over_brake_pedal);
    data(iParticipant).hands_on_wheel_time = data(iParticipant).Time_relative_to_warning_s(data(iParticipant).first_frame_hands_on_wheel);

end

% Now, compute the reaction time to braking and steering.
% Here assume that the steering response happened when the steering angle was more than 5deg.
% Note: 'EGO_steerAngle' is in radians. Use the function deg2rad to convert the angle

for iParticipant = 1 : length(ParticipantList)
    st_an_5=find(rad2deg(data(iParticipant).EGO_steerAngle)>5,1);
    data(iParticipant).steering_time = data(iParticipant).Time_relative_to_warning_s(st_an_5);
    data(iParticipant).braking_time = data(iParticipant).Time_relative_to_warning_s(data(iParticipant).first_frame_braking);

end

% Display response process timing
fprintf('Driver response process\n');
fprintf('-----------------------\n');

for iParticipant = 1 : length(ParticipantList)
    

    fprintf('Driver %d, Reaction time to brake hovering %g [s] \n',data(iParticipant).DriverID,data(iParticipant).hovering_over_brake_pedal_time);
    fprintf('Driver %d, Reaction time to braking action %g [s] \n',data(iParticipant).DriverID,data(iParticipant).braking_time);
    
    fprintf('Driver %d, Reaction time hands on wheel %g [s] \n',data(iParticipant).DriverID,data(iParticipant).hands_on_wheel_time);
    fprintf('Driver %d, Reaction time steering %g [s] \n\n',data(iParticipant).DriverID,data(iParticipant).steering_time);

end

%% Did the drivers successfully evade the front vehicle?
% Or did they crashed against it? To check, you could use the range to the front vehicle.
% Moreover, check whether they collided with the median barrier. Note that the variable
% 'EGO_in_lane' indicates the lane ID the vehicle is on: The Lane IDs are  (...|||4||| 5 | 6 | 7 || : 6 is
% the middle lane, 4 is the median barrier)

fprintf('\nCrash or no crash?\n');
fprintf('------------------\n');

for iParticipant = 1 : length(ParticipantList)
   if any(data(iParticipant).EGO_POV_fender_to_fender_range < 0)
        crash_front_vehicle = 1;
    else
        crash_front_vehicle = 0;
    end
    if any(data(iParticipant).EGO_in_Lane == 4)
        crash_median_barrier = 1;
    else
        crash_median_barrier = 0;
    end
    
    if crash_front_vehicle 
        fprintf('Driver %d crashed against the leading vehicle\n', data(iParticipant).DriverID);
    end

    if crash_median_barrier 
        fprintf('Driver %d crashed against the median barrier\n', data(iParticipant).DriverID);
    end
end

%% QUESTION {enclose your answer in a comment}
% What was the first motor response of the drivers (e.g, braking or
% steering)? 
% Answer:
% For participant ID 2, the first motor response of the driver is braking and for
% participant ID 3, the first motor response of the driver is  steering

%% QUESTION {enclose your answer in a comment}
% If you look again the videos of the driver, what was the very first
% noticeable reaction to the warning? You have annotated only feet and
% hands movements; if you wanted to capture the response process in more detail, what other
% video annotations would you do?
% Answer: 
% The first noticable reation other than stearing and braking is eye
% movement or glance to the warning. To capture the response process in
% more detail eye movement can be annotated

%% QUESTION {enclose your answer in a comment}
% From the timing of the response process, it seems that the driver evaded
% the threat using similar strategies. But, what was the quality of their maneuver? 
% Was it smooth and controlled, or sudden jerky like? To answer the questions, have another look at the videos 
% and at the lateral and longitudinal vehicle measures you plotted before
% Answer:
% By checking the lateral and longitudinal vehicle measures plotted, it is
% obvious that participant id 2 is showing sudden jerky like and participant id is
% smooth and controlled. And also looking to steer angle jerk measure
% partcipant 2 is showing more jerk than participant 3.

%% QUESTION {enclose your answer in a comment}
% The drivers received only an audio warning. Do you think such warning
% strategy was effective to transfer the control from the automation to the
% driver?
% Answer:
% Yes. It is a good strategy to give the audio warning as it brings drivers
% attention and it will be effective to transfer the control from the automation to the
% driver.
