maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;
slamAlg.LoopClosureSearchRadius = 8;
for i=1:10
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scansCopy{i});
    if isScanAccepted
        fprintf('Added scan count  %d \n', i);
    end
end
figure; show(slamAlg); title({'Map of the Environment', 'Pose Graph for Initial 10 Scans'});

%%%%%%%%%%%%%%%%%%% STEP_3 %%%%%%%%%%%%%%%%%%%%%
firstTimeLCDetected = false;
figure;
for i=10:length(scansCopy)
 [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scansCopy{i});
    if ~isScanAccepted
         fprintf('Added scan count  %d \n', i);
       continue;
    end
% visualize the first detected loop closure, if you want to see the complete map building
% process, remove the if condition below
  if optimizationInfo.IsPerformed && ~firstTimeLCDetected
  show(slamAlg, 'Poses', 'off')
  hold on;
  show(slamAlg.PoseGraph);
  hold off;
  firstTimeLCDetected = true;
  drawnow
  end
end

%%%%%%%%%%%%%%%%%%% STEP_4 Visualize the constructed map and trajectory of robot %%%%%%%%%%%%%%%%%%%%% 

title('First loop closure'); 
figure
show(slamAlg);
title({'Final Built Map of the Environment','Trajectory of the Robot'})

%%%%%%%%%%%%%%%%%%% STEP_5 Build occupancy grid %%%%%%%%%%%%%%%%%%%%%

[scansCopy, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scansCopy, optimizedPoses, mapResolution, maxLidarRange);
figure;
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');

% %%%%%%%%%%%%%%%%% STEP_6 (PRM) Probabilistic Road map %%%%%%%%%%%%%%%%%%%%%%
% 
% close all;
% figure
% show(map);title('Occupancy Map');
% %% Goal Setting and Path Planning
% F = getframe
% a = F.cdata
% b = rgb2gray(a);
% c = im2bw(b);
% b = edge(c);
% d = imcomplement(b);
% bwimage = d;
% grid = robotics.BinaryOccupancyGrid(bwimage);
% show(grid);
% robotRadius = 0.1;
% mapInflated copy(map);
% inflate(mapInflated.robotRadius);
% show(mapInflated); title ('Map Inflated');
% impixelinfo;
% [x,y] = ginput(1);
% %x 0.23
% %y=0.05
% prm = robotics.PRM;f
% prm.Map = mapinflated;
% prm.NumNodes = 50;
% prm.ConnectionDistance = 5;
% startLocation = [0,0];
% endLocation = [x, y];
% path = findpath(prm, startLocation, endLocation);
% show(prm); title('PRM');
