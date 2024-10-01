%Load Occupancy Map--------------------------------------------------------

show(map)

% Set start and goal poses.
start = [-4.6,-3.5,0];
goal = [0,0,0];

% Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Show start and goal headings.
r = 0.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
hold off

%Define State Space--------------------------------------------------------

bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.75;

%Plan Path-----------------------------------------------------------------

stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = 1.0;

planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 01.0;
planner.MaxIterations = 10000;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;


rng default
[pthObj, solnInfo] = plan(planner,start,goal);
% pthObj.States(:,2)=-pthObj.States(:,2);

%Plot Path-----------------------------------------------------------------

show(map)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolate and plot path.
interpolate(pthObj,300)

plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

%disp(pthObj.States(:,:))

% Show start and goal in grid map.
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
hold off

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

path = pthObj.States(:,1:2);

robotInitialLocation = path(1,:);
robotGoal = path(end,:);

initialOrientation = pthObj.States(1,3);

robotCurrentPose = [robotInitialLocation initialOrientation]';

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

controller = controllerPurePursuit;

controller.Waypoints = path;

controller.DesiredLinearVelocity = 0.3;

controller.MaxAngularVelocity = 3.5;

controller.LookaheadDistance = 0.3;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = (0.1/3);
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

v1 = 0;
Omega1 = 0;

while( distanceToGoal > goalRadius )

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    %disp([v, omega]);

    Omega1(end+1) = omega ;
    v1(end+1) = v;

    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    disp(vel);
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    % Update the plot
    hold off

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all

    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13])
    ylim([0 13])

    waitfor(vizRate);
end

Omega_out = Omega1';
Omega_out(end+1) = 0;
v_out = v1';
v_out(end+1) = 0;

disp(Omega_out);
disp(v_out);

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end