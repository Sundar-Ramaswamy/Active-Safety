%% ----- Exercise 4 - Naturalistic data analysis -----
% Version: 2021
% Course: TME 192 Active Safety
%         Chalmers
% Author: Alberto Morando (morando@chalmers.se)
%         Alexander Rasch (arasch@chalmers.se)
%         Pierluigi Olleja (ollejap@chalmers.se)
%         Marco Dozza (dozza@chalmers.se)
%
% Group: [14
%         Aparna Ram Suresh Saritha Kumari 
%         Elizabeth Swathika Azariah
%         Sundar Murugan Ramaswamy]
%

close all
clear all
clc

%% Initialization
% Colormap for plotting
cmap = [214,96,77;
    67,147,195]./255;

% to keep track of discarded events
num_discarded_crash_events = 0;
num_discarded_near_crash_events = 0;

% to keep track of the valid event IDs
crash_event_ID = [];
near_crash_event_ID = [];

% these arrays contain the mean speed for each event
crash_mean_speed = [];
near_crash_mean_speed = [];

%% Query data
% Collect the .mat event filenames in the "Events" folder. Here you can use the  ?dir? function
EventFolder = 'Events';
EventFiles = dir(fullfile(EventFolder, '*.mat'));
count1=0; %to count discarded crashes
count2=0; %to count discarded near crashes
total_count = 0; %total count of rear end crashes
% Loop throught the event files.
for i = 1:length(EventFiles)
    
    % Load single events data. Here you can use the  ?load? function
    load(fullfile(EventFolder, EventFiles(i).name));
 
    % ======== YOUR CODE HERE ========

    % Analyse the event only if the incident type is 'Rear-end striking'. Look at
    % the dictionary "ResearcherDictionaryVideoReductionDatav1_1" to understand how to extract
    % this information. Here you can use the ?strcmpi? function
    t1=strcmpi(Data100Car.Video.incident_type,'Rear-end striking');
    
    if (t1==1)
        total_count = total_count + 1;
        % Extract the ?speed? data. Have a look at the dictionary "100CarTimeSeriesDataDictionary_v1_1" to
        % understand how to extract this information
        speed = Data100Car.TimeSeries(:,5);
        
        % Check speed quality with the function "isGoodQuality".
        % If the quality is not good we will skip the event.
        
        if (isGoodQuality(speed)) 
            
            % Extract time [s]. Make it start from 0
            time =  Data100Car.TimeSeries(:,3)- Data100Car.TimeSeries(1,3);
            
            % The speed signal may be corrupted (missing data or sensor
            % error). Here we will implement a simple, yet effective, strategy for dealing
            % with this issue. We will perform a linear interpolation to
            % recover the missing data. Specifically, interpolate the
            % values that are less or equal to zero.
            
            % Make a copy of the vector
            speed_fix = speed;
            
            % Interpolate the speed
            speed_fix(speed_fix<=0) = interp1(time(speed_fix>0),speed_fix(speed_fix>0),time(speed_fix<=0));
            %speed_fix = fillmissing(speed_fix,'linear','SamplePoints',time);
           
           
            % extract the speed only during the incident. Yout could use `find`
            idx_start_incident = find(Data100Car.TimeSeries(:,2) == Data100Car.Video.start);
            idx_end_incident = find(Data100Car.TimeSeries(:,2) == Data100Car.Video.end);
            speed_incident = speed_fix(idx_start_incident:idx_end_incident);
            
            
            
            % Check if the event was a `Crash` or `Near Crash`. Depending
            % on the incident category, store the event ID and the mean
            % speed in the right variable.
            % Take a look at the dictionary "ResearcherDictionaryVideoReductionDatav1_1" to understand how to extract
            % this information.
            if (strcmpi(Data100Car.Video.severity,'Crash')) % If `Crash`
                % save the ID of the event and append it to the vector
                crash_event_ID = [crash_event_ID; Data100Car.ID];
                % compute and append average speed to the vector
                speed_crash = Data100Car.TimeSeries(:,5);
                time_crash =  Data100Car.TimeSeries(:,3)-Data100Car.TimeSeries(1,3);
                speed_crash(speed_crash<=0) = interp1(time_crash(speed_crash>0),speed_crash(speed_crash>0),time_crash(speed_crash<=0));
                idx_start_incident_c = find(Data100Car.TimeSeries(:,2) == Data100Car.Video.start);
                idx_end_incident_c = find(Data100Car.TimeSeries(:,2) == Data100Car.Video.end);
                speed_incident_c = speed_fix(idx_start_incident_c:idx_end_incident_c);
                crash_mean_speed = [crash_mean_speed; nanmean(speed_incident_c)];
                
            elseif (strcmpi(Data100Car.Video.severity,'Near Crash')) % If `Near Crash`
                % save the ID of the event and append it to the vector
                near_crash_event_ID = [near_crash_event_ID; Data100Car.ID];
                % compute and append average speed to the vector
                speed_near_crash = Data100Car.TimeSeries(:,5);
                time_near_crash =  Data100Car.TimeSeries(:,3)-Data100Car.TimeSeries(1,3);
                speed_near_crash(speed_near_crash<=0) = interp1(time_near_crash(speed_near_crash>0),speed_near_crash(speed_near_crash>0),time_near_crash(speed_near_crash<=0));
                idx_start_incident_nc = find(Data100Car.TimeSeries(:,2) == Data100Car.Video.start);
                idx_end_incident_nc = find(Data100Car.TimeSeries(:,2) == Data100Car.Video.end);
                speed_incident_nc = speed_fix(idx_start_incident_nc:idx_end_incident_nc);
                near_crash_mean_speed = [near_crash_mean_speed; nanmean(speed_incident_nc)];
            end
            
        else
            % If the data quality is low, we discard the event. However, we
            % will keep track of how many events were discarded per category.
            if (strcmpi(Data100Car.Video.severity,'Crash'))% If `Crash`
                count1=count1+1;
                num_discarded_crash_events = count1;
            elseif (strcmpi(Data100Car.Video.severity,'Near Crash')) % If `Near Crash`
                count2=count2+1;
                num_discarded_near_crash_events = count2;
            end
        end
    end
end


%% Proportion [%] of missing data.
% ======== YOUR CODE HERE ========
% from the number of discarded events and the number of valid ones, compute the percentage of
% missing data for the crash and near-crash events
percentageMissingDataCrashes = (num_discarded_crash_events/total_count)*100;
percentageMissingDataNearCrashes = (num_discarded_near_crash_events/total_count)*100;

fprintf('Percentage missing data for crash events = %3.2f %%\n', percentageMissingDataCrashes)
fprintf('Percentage missing data for near-crash events = %3.2f %%\n', percentageMissingDataNearCrashes)

%% QUESTION
% Is the percentage of missing data you found usual for a naturalistic dataset?
% (Enclose your answer in a comment)
% A data loss of 10 to 20% is not unusual(from Marco's lecture slides). As the percentage of missing
% data is 1% for crash events and 11.78% for near crash events it is usual.

%% Statistics: compare the samples. Is the average speed is significantly lower in crashes than in near crashes?
% Compare the speed for Near-crashes and crashes with a t-test.
% Before running the t-test, you should verify that the data meet the
% assumption of normality. If the data are not normally distribuited, the result from the t-test may be misleading.
% To do so, plot the average speed values for the crash and near-crash
% events with a histogram (use function ?histogram?).
bins = 0:5:100;
x_pdf = [0:0.1:max(bins)];

figure('name', 'original data')
hold on

% Distribution mean speed for near-crashes
histogram(near_crash_mean_speed, bins,...
    'normalization', 'pdf', ...
    'facecolor', cmap(1, :))

% Fit a normal distribution to the data
y = pdf(fitdist(near_crash_mean_speed,'Normal'), x_pdf);
plot(x_pdf, y, '-', 'color', cmap(1, :))

% Distribution mean speed for near-crashes
histogram(crash_mean_speed, bins,...
    'normalization', 'pdf',...
    'facecolor', cmap(2, :))

% Fit a normal distribution to the data
y = pdf(fitdist(crash_mean_speed,'Normal'), x_pdf);
plot(x_pdf, y, '-', 'color', cmap(2, :))
xlim([0 100])
xlabel('Mean speed [mph]')
ylabel('Probability density')

legend({'Near crashes', 'Fitted Normal distribution Near Crashes',...
    'Crashes', 'Fitted Normal distribution Crashes'})




%% Data transformation
% If a measurement variable does not quite fit a normal distribution, you should try a data transformation.
% For example, you could try to apply the square root (`sqrt`). Does the
% transformation make the data fit the normality assumption better?

% ======== YOUR CODE HERE ========
near_crash_mean_speed_tranformed = sqrt(near_crash_mean_speed);
crash_mean_speed_tranformed = sqrt(crash_mean_speed);

bins = 0:0.5:10;
x_pdf = [0:0.1:max(bins)];

figure('name', 'trasnformed data')
hold on

% Distribution mean speed for near-crashes
histogram(near_crash_mean_speed_tranformed, bins,...
    'normalization', 'pdf', ...
    'facecolor', cmap(1, :))

% Fit a normal distribution to the data
y = pdf(fitdist(near_crash_mean_speed_tranformed,'Normal'), x_pdf);
plot(x_pdf, y, '-', 'color', cmap(1, :))

% Distribution mean speed for near-crashes
histogram(crash_mean_speed_tranformed, bins,...
    'normalization', 'pdf',...
    'facecolor', cmap(2, :))

% Fit a normal distribution to the data
y = pdf(fitdist(crash_mean_speed_tranformed,'Normal'), x_pdf);
plot(x_pdf, y, '-', 'color', cmap(2, :))
xlim([0 10])
xlabel('Mean speed [mph]')
ylabel('Probability density')

legend({'Near crashes', 'Fitted Normal distribution Near Crashes', ...
    'Crashes', 'Fitted Normal distribution Crashes'})

%% t-test
% Hyhothesis to test ?Average velocity is higher in rear-end striking near-crashes than in
% rear-end striking crashes?. Run the ttest. Use the function ?ttest2?. You should set the option `tail` according to the hypothesis
% you are testing. Be careful on the order of the data you input.

% ======== YOUR CODE HERE ========
[H,P, ~, ~] = ttest2(crash_mean_speed, near_crash_mean_speed, 'tail', 'right', 'Vartype','unequal')
%% QUESTION
% Does the ttest support the hyp. that ?Average velocity is higher in rear-end striking near-crashes than in rear-end striking crashes??
% (Enclose your answer in a comment)
% Given Null hypothesis: “Average velocity is higher in rear-end striking
% near-crashes than in rear-end striking crashes”
% By checking the alternative hypothesis(contradictory statement of the null hypothesis), 
% we get values for P = 1 and H = 0: Hence the null hypothesis is true.

%% Explore the narrative of the valid crash events
% The list of event IDs to load are in the vector ?crash_event_ID?.

% Loop over the event files. Like we did in the beginning of this script
for i = 1:length(EventFiles)
    
    % ======== YOUR CODE HERE ========
    % Load single events data. Here you can use the  ?load? function
    load(fullfile(EventFolder, EventFiles(i).name));
    
    % Check if the event ID is included in the vector `crash_event_ID`. You
    % can use `ismember`
    if (ismember(Data100Car.ID,crash_event_ID) == 1)
        
        % Print the narrative on the console.
        narrative = Data100Car.Narratives;
        fprintf('Event %d: %s\n', Data100Car.ID, narrative{1})
    end
end

%% QUESTION
% By reading the narrative of the event, what do you think is
% the most common scenario and critical reason that lead to a crash?
% (Enclose your answer in a comment)
% The most common reason that led to a rear end crash in the above cases was Inopportune
% glance or driver distraction.
