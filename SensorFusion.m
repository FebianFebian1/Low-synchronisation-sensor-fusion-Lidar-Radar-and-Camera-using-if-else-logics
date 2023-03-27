close all
clc
clear all

%% Loading Data and adjust the low level synch for all sensors

% experiment result (range and time) in mat file
load('expCam.mat');
load('expRad.mat');
load('expLid.mat');
load('expGT.mat');

% Adjust this for low level sync
t_off = 5.5; %time offset in respect to camera
lt_off = 3.6;
gt_off = 6.7;
d_off = 51; % constant offset cam and radar
ld_off = 78; % constant offset lidar to radar
lux = 100;

radar_time = (time_frame)+t_off;
time = time+lt_off;
distance = (distance*1000);
range = (range*1000) + ld_off; 

timet=exp1_groundtruth(:,1)+gt_off;
distancet=exp1_groundtruth(:,2)*1000;


%% Getting more data points between each sensor data points (Linear Interp)
tic

% Fusion Parameter
timewindow = 0.1; % desired update timeframe
step = 0.01; % step for interpolation datapoints

[LidarRange, LidarTimes] = interpolatePoints(range, time, step);
[RadarRange, RadarTimes] = interpolatePoints(distance', radar_time, step);
[CameraRange, CameraTimes] = interpolatePoints(dist_point_coord_def, tempos_def, step);

%Creating combine array of range v times of the new data
Lidar = [LidarRange; LidarTimes];
Radar = [RadarRange; RadarTimes];
Camera = [CameraRange; CameraTimes];
    

%% Fusion attempts

if RadarTimes(:,end) > LidarTimes(:,end)
    if RadarTimes(:,end) > CameraTimes(:,end)        
        endtime = RadarTimes(:,end);
    else
        endtime = CameraTimes(:,end)
    end
else
    if LidarTimes(:,end) > CameraTimes(:,end)
        endtime = LidarTimes(:,end);
    else
        endtime = CameraTimes(:,end);
    end
end

%Processing par 
index = 1;
NewTimes = 0:timewindow:endtime; % Array Times for plotting the fusion distance

for FusionTimes = 0:timewindow:endtime
    % Finding the indexing of range in respect to the fusion time
    [indlx, indly] = find(Lidar(2,:)<=FusionTimes+0.04 & Lidar(2,:)>=FusionTimes-0.04);
    [indrx, indry] = find(Radar(2,:)<=FusionTimes+0.04 & Radar(2,:)>=FusionTimes-0.04);
    [indcx, indcy] = find(Camera(2,:)<=FusionTimes+0.04 & Camera(2,:)>=FusionTimes-0.04);
    
    % Logic for combining results based on the availability of the data
    if (isnan(indly) & isnan(indry) & isnan(indcy))
        continue
    elseif isnan(indly) & isnan(indcy)
        NewDistance(index) = NewRange(Radar,indry);
        index = index+1;
    elseif isnan(indly) & isnan(indry)
        NewDistance(index) = NewRange(Camera,indcy);
        index = index+1;
    elseif isnan(indry) & isnan(indcy)
        NewDistance(index) = NewRange(Lidar,indly);
        index = index+1;
    elseif isnan(indly)
        NewDistance(index) = 0.6*NewRange(Radar,indry) + 0.4*NewRange(Camera,indcy);
        index = index+1;
    elseif isnan(indry)
        NewDistance(index) = 0.6*NewRange(Lidar,indly)+ 0.4*NewRange(Camera,indcy);
        index = index+1;
    elseif isnan(indcy)
        NewDistance(index) = 0.6*NewRange(Radar,indry) + 0.4*NewRange(Lidar,indly);
        index = index+1;
    else
        if lux <= 50
            if NewRange(Lidar,indly) > 3500 |NewRange(Lidar,indly) < 1000        
                NewDistance(index) = 0.15*NewRange(Lidar,indly)+0.05*NewRange(Camera,indcy)+0.8*NewRange(Radar,indry);
                index = index+1;
            elseif NewRange(Radar,indry) < 700       
                NewDistance(index) = 0.05*NewRange(Lidar,indly)+0.05*NewRange(Camera,indcy)+0.9*NewRange(Radar,indry);
                index = index+1;
            else
                NewDistance(index) = 0.25*NewRange(Lidar,indly)+0.05*NewRange(Camera,indcy)+0.7*NewRange(Radar,indry);
                index = index+1;
            end
        else
            if NewRange(Lidar,indly) > 3500 |NewRange(Lidar,indly) < 1000        
                NewDistance(index) = 0.4*NewRange(Lidar,indly)+0.1*NewRange(Camera,indcy)+0.5*NewRange(Radar,indry);
                index = index+1;
            elseif NewRange(Radar,indry) < 1000       
                NewDistance(index) = 0.5*NewRange(Lidar,indly)+0.1*NewRange(Camera,indcy)+0.4*NewRange(Radar,indry);
                index = index+1;
            else
                NewDistance(index) = 0.45*NewRange(Lidar,indly)+0.1*NewRange(Camera,indcy)+0.45*NewRange(Radar,indry);
                index = index+1;
            end
        end
    end  
end

%Fusion Result
Fusion = [NewTimes; NewDistance];

time_compare = toc;
time_compare

%% Plotting the all of the results
figure()
plot(tempos_def,(dist_point_coord_def-d_off));hold on
plot(radar_time(:,2:end),distance(2:end,:));hold on
plot(time,range);hold on
plot(timet, distancet,'-o');hold on
plot(NewTimes(1,:),NewDistance(1,:),'-o','LineWidth',1);
grid minor; grid on;
legend('Camera Result','Radar Result','Lidar Result','Ground Truth','Fusion Result')
title('Experiment 5 on 13/03 fusion result ')
ylabel('Distance in mm')
xlabel('Time in s')


%% Comparing Fusion results with Ground truth

for i = 1:27
    ind = find(abs(Fusion(1,:)-round(timet(i,1),1))<0.001);
    
    if isnan(Fusion(2,ind))
        continue
    else
        DisCom(i)= (Fusion(2,ind)-distancet(i,1))/1000;
        PDisCom(i)= abs(Fusion(2,ind)-distancet(i,1))/distancet(i,1);
    end
end

Mean_Squared_Error = ((1/length(DisCom))*sum(DisCom))^2
Mean_Percentage_Error = 100*(1/length(DisCom))*sum(PDisCom)
