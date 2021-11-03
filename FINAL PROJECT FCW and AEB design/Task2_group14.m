%% ----- Task 2 - ACTIVE SAFETY PROJECT -----
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
load('RadarData.mat');

Table = zeros(82, 6);

%New struct creation
RadarData_edit = cell2struct(struct2cell(RadarData),fieldnames(RadarData));
field_1 = 'TestPerson';
RadarData_edit = rmfield(RadarData_edit,field_1);
field_2 = 'FileName';
RadarData_edit = rmfield(RadarData_edit,field_2);

% emptying test runs with all NaN values
for i=1:numel(RadarData_edit)  
    rowsWithNaN = any(isnan(double(RadarData_edit(i).RadarRange)),2); 
    index = double(rowsWithNaN); 
    if size(rowsWithNaN,1) == sum(rowsWithNaN)
        RadarData_edit(i).RadarRange(isnan(RadarData_edit(i).RadarRange...
            (:,1)),:) = [];
        RadarData_edit(i).RadarRangeRate = [];
        RadarData_edit(i).RadarAccel = [];
        RadarData_edit(i).VehicleSpeed = [];
        RadarData_edit(i).VehicleYawRate = [];
        RadarData_edit(i).RadarTime = [];
        RadarData_edit(i).VehicleTime = [];
    end 
end

%removing all empty test runs
empty_elems = arrayfun(@(s) all(structfun(@isempty,s)), RadarData_edit);
RadarData_edit(empty_elems) = [];


%synchrozing time 
for i=1:numel(RadarData_edit)
    RadarData_edit(i).RadarTime = RadarData_edit(i).RadarTime-0.2;
end

%removing unwanted extra values of vehicle speed, yaw rate and vehicle time
for i=1:numel(RadarData_edit)
    index_radar = length(RadarData_edit(i).RadarRange(:,1));
    useful_values = 1:index_radar;
    if index_radar < length(RadarData_edit(i).VehicleSpeed)
        RadarData_edit(i).VehicleSpeed = RadarData_edit(i).VehicleSpeed...
            (useful_values,1);
        RadarData_edit(i).VehicleYawRate = RadarData_edit(i).VehicleYawRate...
            (useful_values,1);
        RadarData_edit(i).VehicleTime = RadarData_edit(i).VehicleTime...
            (useful_values,1);
    end
end

%Finding Vehicle Acceleration and interpolating radar range and radar acceleration values
for i=1:numel(RadarData_edit)
    dx = RadarData_edit(i).VehicleSpeed(2:end) - RadarData_edit(i).VehicleSpeed(1:end-1);
    dt = RadarData_edit(i).VehicleTime(2:end) - RadarData_edit(i).VehicleTime(1:end-1);
    RadarData_edit(i).VehicleAccel =  [0; dx./dt]; 
    RadarData_edit(i).MeanAccel = mean(RadarData_edit(i).VehicleAccel);
    RadarData_edit(i).MeanAccel = -(abs(RadarData_edit(i).MeanAccel));
    RadarData_edit(i).RadarRange = fillmissing(RadarData_edit(i).RadarRange,...
        'linear','SamplePoints',RadarData_edit(i).RadarTime); %interpolating RadarRange
    RadarData_edit(i).RadarAccel = fillmissing(RadarData_edit(i).RadarAccel,...
        'linear','SamplePoints',RadarData_edit(i).RadarTime); %interpolating RadarAccel
   
end

%Interpolating RadarRangeRate to get rid of NaNs
for y=1:numel(RadarData_edit)
    NaNs = find(isnan(RadarData_edit(y).RadarRangeRate));
    non_empty_indices = find(~isnan(RadarData_edit(y).RadarRangeRate));
    RadarData_edit(y).RadarRangeRate(isnan(RadarData_edit(y).RadarRangeRate)) = ...
        -RadarData_edit(y).VehicleSpeed(NaNs); %RadarRangeRate and VehicleSpeed must be the same
end

for z=1:numel(RadarData_edit)
    RadarData_edit(z).RadarRangeRate = RadarData_edit(z).RadarRangeRate; %reversing abs values
% Braking algorithm    
    [peaks,idx]=findpeaks(RadarData_edit(z).VehicleSpeed(:,1));
    cpt_VehAcc = findchangepts(RadarData_edit(z).VehicleAccel,'MaxNumChanges',2);
    cpt_YawRate = findchangepts(RadarData_edit(z).VehicleYawRate,'MaxNumChanges',2);
    cpt_RadRate = findchangepts(RadarData_edit(z).RadarRangeRate,'MaxNumChanges',2);
%test run removal part
    if z==7   % Very low speed at brake onset
        continue
    elseif z==20   % abrupt changes in yaw rate which indicates steering action performed by the driver
        continue
    elseif z==46   % Very high range and very low speed at brake onset
        continue
    elseif z==51   % Very high range and very low speed at brake onset
        continue
    end  
    
%Brake onset    
    if length(cpt_VehAcc) == 1   %if there is only one abrupt change point 
%         if RadarData_edit(z).MeanAccel < -0.1 
%             bo_1 = find(RadarData_edit(z).VehicleAccel <  2.3 * RadarData_edit(z).MeanAccel); %if the acceleration value goes below the threshold
%             bo = bo_1(1);  %first instance when the acceleration value goes below the threshold
        if RadarData_edit(z).MeanAccel < -0.0001 %for any mean acceleration value
            [bo_val,bo_1] = min(RadarData_edit(z).VehicleAccel); %finding value and index of min acceleration
            for j = 1:bo_1
                if RadarData_edit(z).VehicleAccel(j) > 0.24*min(RadarData_edit(z).VehicleAccel) %finding abrubt change point manually using a threshold
                    bo = j; %index of abrupt change point
                end
            end
        end
    elseif isempty(cpt_VehAcc) == true  %if there are no abrupt change points 
        if RadarData_edit(z).MeanAccel < -0.0001 %for any mean acceleration value
            [bo_val,bo_1] = min(RadarData_edit(z).VehicleAccel); %finding value and index of min acceleration
            for j = 1:bo_1
                if RadarData_edit(z).VehicleAccel(j) > 0.24*min(RadarData_edit(z).VehicleAccel) %finding abrubt change point manually using a threshold
                    bo = j; %index of abrupt change point
                end
            end
        end
    elseif isempty(cpt_VehAcc)==false    % if there are abrupt changes in vehicle acceleration
        if RadarData_edit(z).VehicleAccel(cpt_VehAcc(1)) > 0 || RadarData_edit(z).VehicleAccel(cpt_VehAcc(2)) > 0 %if the change point value of acceleration is positive
            [bo_val,bo_1] = min(RadarData_edit(z).VehicleAccel); %finding value and index of min acceleration
            for j = 1:bo_1
                if RadarData_edit(z).VehicleAccel(j) > 0.24*min(RadarData_edit(z).VehicleAccel) %finding abrupt change point manually using a threshold
                    bo = j; %index of abrupt change point
                end
            end
        elseif cpt_VehAcc(2)-cpt_VehAcc(1)>150 %if the change points are very far away
            [bo_val,bo_1] = min(RadarData_edit(z).VehicleAccel); %finding value and index of min acceleration
            for j = 1:bo_1
                if RadarData_edit(z).VehicleAccel(j) > 0.24*min(RadarData_edit(z).VehicleAccel) %finding abrupt change point manually using a threshold
                    bo = j; %index of abrupt change point
                end
            end    
        else
            bo = cpt_VehAcc(1);
        end           
    end
%Brake release
    if length(cpt_VehAcc) == 1   %if there is only one abrupt change point 
        br_1 = find(RadarData_edit(z).VehicleAccel > 2.3 * RadarData_edit(z).MeanAccel); %if the acceleration value goes above the threshold
            for i = 1:numel(br_1)
                if br_1(i) > bo_1(1)  %first instance when the acceleration value goes above the threshold
                    br = br_1(i);
                    break
                end
            end
    elseif isempty(cpt_VehAcc) == true   %if there are no abrupt change points 
        br_1 = find(RadarData_edit(z).VehicleAccel > 2.3 * RadarData_edit(z).MeanAccel); %if the acceleration value goes above the threshold
            for i = 1:numel(br_1)
                if br_1(i) > bo_1(1)  %first instance when the acceleration value goes above the threshold
                    br = br_1(i);
                    break
                end
            end
    elseif isempty(cpt_VehAcc)==false    % if there are abrupt changes in vehicle acceleration
        if RadarData_edit(z).VehicleAccel(cpt_VehAcc(1)) > 0 || RadarData_edit(z).VehicleAccel(cpt_VehAcc(2)) > 0 %if the change point value of acceleration is positive
            [br_val,br_1] = min(RadarData_edit(z).VehicleAccel); %finding value and index of min acceleration
            for i = br_1:numel(RadarData_edit(z).VehicleAccel)
                if RadarData_edit(z).VehicleAccel(i) > 0.24*min(RadarData_edit(z).VehicleAccel) %finding abrupt change point manually using a threshold
                    br = i;       %index of abrupt change point
                    break
                end
            end
        elseif cpt_VehAcc(2)-cpt_VehAcc(1)>150 %if the change points are very far away
            [br_val,br_1] = min(RadarData_edit(z).VehicleAccel); %finding value and index of min acceleration
            for i = br_1:numel(RadarData_edit(z).VehicleAccel)
                if RadarData_edit(z).VehicleAccel(i) > 0.24*min(RadarData_edit(z).VehicleAccel) %finding abrupt change point manually using a threshold
                    br = i;       %index of abrupt change point
                    break
                end
            end  
        else
            br = cpt_VehAcc(2);
        end
    end
    
    
    
    figure
    subplot(2,1,1)
        hold on ; grid on
        cla 
        plot(RadarData_edit(z).RadarTime, RadarData_edit(z).RadarRange)
        xlabel('RadarTime [s]')
        ylabel('RadarRange [m]')
        title('RadarRange')
    subplot(2,1,2)
        hold on ; grid on
        cla 
        plot(RadarData_edit(z).RadarTime, RadarData_edit(z).RadarRangeRate)
        xlabel('VehicleTime [s]')
        ylabel('RadarRangeRate [m/s]')
        title('RadarRangeRate')
    figure
    subplot(3,1,1)
        hold on ; grid on
        cla 
        plot(RadarData_edit(z).VehicleTime, RadarData_edit(z).VehicleSpeed)

        xline(RadarData_edit(z).VehicleTime(bo), '-k', 'Brake onset') 
        xline(RadarData_edit(z).VehicleTime(br), '-k', 'Brake release')

        xlabel('RadarTime [s]')
        ylabel('VehicleSpeed [m/s]')
        title('VehicleSpeed')
    subplot(3,1,2)
        hold on ; grid on
        cla 
        plot(RadarData_edit(z).VehicleTime, RadarData_edit(z).VehicleAccel)
        xline(RadarData_edit(z).VehicleTime(bo), '-k') 
        xline(RadarData_edit(z).VehicleTime(br), '-k')
        xlabel('VehicleTime [s]')
        ylabel('VehicleAccel [m/s^2]')
        title('VehicleAccel')
    subplot(3,1,3)
        hold on ; grid on
        cla 
        plot(RadarData_edit(z).VehicleTime, RadarData_edit(z).VehicleYawRate)
%         if isempty(cpt_YawRate) == false
%             xline(RadarData_edit(z).VehicleTime(cpt_YawRate(1)), '-r')
%         end
        xlabel('VehicleTime [s]')
        ylabel('VehicleYawRate [rad/s]')
        title('VehicleYawRate')
    TestRuns = z;
    MeanAcceleration = mean(RadarData_edit(z).VehicleAccel(bo:br),'all');
    MinimumAcceleration = min(RadarData_edit(z).VehicleAccel);
    Speed_BrakeOnset = RadarData_edit(z).VehicleSpeed(bo);
    Range_BrakeOnset = RadarData_edit(z).RadarRange(bo);
    TTC = RadarData_edit(z).RadarRange(bo)./RadarData_edit(z).VehicleSpeed(bo);
    Table(z, :) = [TestRuns MeanAcceleration MinimumAcceleration Speed_BrakeOnset Range_BrakeOnset TTC];
       
end

ind = find(sum(Table,2)==0) ;
Table(ind,:) = [] ; % Rows with values zero are those which are removed
Safety_Metrics = array2table(Table);
Safety_Metrics.Properties.VariableNames = {'Test Runs' 'Mean_Acceleration' 'Min_Acceleration' 'Speed at Brake Onset' 'Range at Brake Onset' 'TTC'};

% SAFETY_METRICS is the final table
save('Safety_Metrics.mat', 'Safety_Metrics')

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    

  



    