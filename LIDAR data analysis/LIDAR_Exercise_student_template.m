% Student template script for TME192 LIDAR exercise
% Group 14 
% Group Members : Aparna Ram Suresh Saritha Kumari 
%                 Elizabeth Swathika Azariah
%                 Sundar Murugan Ramaswamy


% Load the data if not available. you may have to set specific path
if ~exist('oData')
    load('oData')
end

% Initiate a plot
fig=figure(1);

% Set the coordinates for what to show
fPlotCoordsX=[6407050,6407120];
fPlotCoordsY=[1276550,1276650];

% Initiate an AVI. 
% STUDENT: YOU HAVE TO CHANGE THE PATH!
aviobj = VideoWriter(['Z:\Active_safety\Exercise_1\film_' datestr(now,30) '.avi'],'MPEG-4');
open(aviobj);

% Loop through all times in the Sensor Fused data
for iIndex=1:length(oData.iTimeSF)

   % Get the specific time for this index from Sensor Fusion data		
   time=oData.iTimeSF(iIndex);
   
   % Find the closest LIDAR time corresponding to the Sensor Fusion time 
   iLIDARIndex=find(oData.iLidarTime>time,1);

   
   % FROM HERE ON STUDENT CODE - The code within this is what should be
   % pasted into the "[fXechoGlobal, fYechoGlobal] = coordinateProjection(oData, iIndex)" function
   
   % Do the translations and coordinate transformations to extract the
   %   LIDAR reflections in the coordinate system of RT90 (GPS antenna
   %   mounting position)
    
   % Add the RT90 position (global coordinates from GPS), but in order to 
   %  be able to add them vehicle data it will have to be projected on the 
   %  RT90 coordinate system using the heading.

   % Add to the RT90 cartesian coordinate system. When the code for
   % the two global coordinates is ready, the output should be in 
   % these two variables. They should be the final output pasted 
   % into the function in the Matlab Grader. If you have multiple
   % lines of code to create the two variables, all should be
   % pasted into Matlab Grader.
   
   gps_x=(oData.fLIDAR_X{iIndex} + oData.fLIDARposX - oData.fGPSposX);
    gps_y=oData.fLIDAR_Y{iIndex} + oData.fLIDARposY;
    
    angle = deg2rad(180) - ((deg2rad(90)) + oData.fHeadingSF(iIndex));
    
    hyp_s = gps_y ./ sin(angle);
    adj_s = cos(angle) .* hyp_s;
    
    hypo = gps_x - adj_s;
    oppo = sin(oData.fHeadingSF(iIndex)) .* hypo;
    adj = cos(oData.fHeadingSF(iIndex)) .* hypo;
    
    y_value = oppo + hyp_s;
    
    fXechoGlobal = adj + oData.fXRT90SF(iIndex);
    fYechoGlobal = (y_value + oData.fYRT90SF(iIndex));
   
   
     

    % END OF STUDENT CODE (if you want, more can be added) 

   % Plot the lidar in RT90 coodrinate system	   
   plot(fXechoGlobal,fYechoGlobal,'.')

   % Plot the vehicle position (the GPS antenna) too
   plot(oData.fXRT90SF(iIndex),oData.fYRT90SF(iIndex),'.r','MarkerSize',30)
   
   % Add your name to the plot
   %%% STUDENT: You should change this (X) should be your group number
   text(6407100,1276560,'Group 14')
   
   % Set the axis of the plot
   axis([fPlotCoordsX fPlotCoordsY])
   hold on;
   
   % Get it as an avi-frame
   F = getframe(fig);
   % Add the frame to the avi
   writeVideo(aviobj,F);
   %aviobj = addframe(aviobj,F);
   
end

% Close the AVI from Matlab
close(aviobj);
