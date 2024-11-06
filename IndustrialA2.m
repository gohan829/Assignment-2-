clear all;
clc;
hold on;
axis equal;

% Global variable declaration for stopping conditions
global stopFlag;
stopFlag = false;

Env = EnvironmentSetup();

% Define constants for the robot's actions
groundHeight = 0;
liftHeight = 0.2;
safeZLimit = 0.05;
numSteps = 35;

% Initializes and configure the robot
robot = LinearUR3e(transl(0.6, -0.4, 0.05));
r2 = LinearUR5(transl(0.6,1.3,0.05));
axis([-3, 3, -3, 3, 0, 2]);

% Store the initial robot position
initialRobotPos = robot.model.getpos();
initialRobotPosR2 = r2.model.getpos();

% Define the positions where the robot will pick up and drop off the apple
pickPosition = [0.5, 0, 0.1];
dropPosition = [0.8, -1.2, 0.7];

% Place the apple visually at the pick-up position
apple = PlaceObject('apple5.ply', pickPosition);

% Gets the robot's initial joint angles (starting position)
currentRobotPos = robot.model.getpos();

% Initialize the grippers for the first robot
finger1 = Gripper(robot.model.fkine(robot.model.getpos()));
finger2 = Gripper(robot.model.fkine(robot.model.getpos()));

% Initialize grippers for the second robot (r2)
finger3 = Gripper(r2.model.fkine(r2.model.getpos()));
finger4 = Gripper(r2.model.fkine(r2.model.getpos()));

qopen = zeros(1,3);
qclose = [-0.2513 0.6912 -0.4398];
qmatc = jtraj(qopen, qclose, numSteps);
qmato = jtraj(qclose, qopen, numSteps);

% Store path for Robot 1
pathRobot1 = [];

% Launch the GUI
RobotControlGUI(robot, finger1, finger2, qmatc, qmato, initialRobotPos);

% Reset simulation function to return to step 1
function resetSimulation()
    stopFlag = true;
    pause(0.1);

    stopFlag = false;
    startMainLoop = true;

    % Reset robot and grippers to initial positions
    currentRobotPos = initialRobotPos;
    smoothTrajectoryBack = jtraj(robot.model.getpos(), initialRobotPos, numSteps);
    for step = 1:numSteps
        if stopFlag, break; end
        robot.model.animate(smoothTrajectoryBack(step, :));

        % Update gripper positions along with robot during reset
        tr = robot.model.fkine(smoothTrajectoryBack(step, :));
        finger1.model.base = tr.T * trotx(pi/2);
        finger2.model.base = tr.T * trotz(pi) * trotx(pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.animate(finger2.model.getpos());

        axis equal;
        drawnow();
    end

    % Open the gripper at the initial position
    for step = 1:numSteps
        finger1.model.animate(qmato(step, :));
        finger2.model.animate(qmato(step, :));
        drawnow();
    end
% Define waypoints for robot1 between pick and drop positions
waypoint1 = [0.6, -0.3, 0.5];  
waypoint2 = [0.7, -0.6, 0.6];  
    disp('Simulation Reset - Returning to Step 1');
end

%% Main Loop: Move the apple from pick-up position to drop-off position

% Step 1: Move the robot above the pick-up position
if ~stopFlag
    disp('Moving to the apple at pick-up position');
    targetAbovePick = transl(pickPosition(1), pickPosition(2), pickPosition(3) + liftHeight) * trotx(pi);
    newRobotPos = robot.model.ikcon(targetAbovePick, currentRobotPos);
    smoothTrajectory = jtraj(currentRobotPos, newRobotPos, numSteps);
    for step = 1:numSteps
        robot.model.animate(smoothTrajectory(step, :));

        % Collect path for Robot 1
        tr = robot.model.fkine(smoothTrajectory(step, :));
        pathRobot1 = [pathRobot1; tr.t'];

        % Move the grippers with the robot
        finger1.model.base = tr.T * trotx(pi/2);
        finger2.model.base = tr.T * trotz(pi) * trotx(pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.animate(finger2.model.getpos());

        axis equal;
        drawnow();
    end
end
currentRobotPos = newRobotPos;

% Step 2: Move down to pick up the apple
if ~stopFlag
    disp('Moving down to pick up the apple');
    targetPick = transl(pickPosition(1), pickPosition(2), pickPosition(3) + safeZLimit) * trotx(pi);
    newRobotPos = robot.model.ikcon(targetPick, currentRobotPos);
    smoothTrajectory = jtraj(currentRobotPos, newRobotPos, numSteps);
    for step = 1:numSteps
        robot.model.animate(smoothTrajectory(step, :));

        % Collect path for Robot 1
        tr = robot.model.fkine(smoothTrajectory(step, :));
        pathRobot1 = [pathRobot1; tr.t'];

        % Move the grippers with the robot
        finger1.model.base = tr.T * trotx(pi/2);
        finger2.model.base = tr.T * trotz(pi) * trotx(pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.animate(finger2.model.getpos());

        axis equal;
        drawnow();
    end
end
currentRobotPos = newRobotPos;

% Step 3: Lift the apple after picking it up
if ~stopFlag
    disp('Lifting the apple');
    targetLift = transl(pickPosition(1), pickPosition(2), pickPosition(3) + liftHeight) * trotx(pi);
    newRobotPos = robot.model.ikcon(targetLift, currentRobotPos);
    smoothTrajectory = jtraj(currentRobotPos, newRobotPos, numSteps);
    for step = 1:numSteps
        robot.model.animate(smoothTrajectory(step, :));

        % Collect path for Robot 1
        tr = robot.model.fkine(smoothTrajectory(step, :));
        pathRobot1 = [pathRobot1; tr.t'];

        % Move the apple with the robot
        delete(apple);
        apple = PlaceObject('apple5.ply', [tr.t(1), tr.t(2), tr.t(3)]);

        % Move the grippers with the robot
        finger1.model.base = tr.T * trotx(pi/2);
        finger2.model.base = tr.T * trotz(pi) * trotx(pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.animate(finger2.model.getpos());

        axis equal;
        drawnow();
    end
end
currentRobotPos = newRobotPos;

% Step 4: Move the robot to the drop-off position
if ~stopFlag
    disp('Moving to the drop-off position');
    targetAbovePlace = transl(dropPosition(1), dropPosition(2), dropPosition(3) + liftHeight) * trotx(pi);
    newRobotPos = robot.model.ikcon(targetAbovePlace, currentRobotPos);
    smoothTrajectory = jtraj(currentRobotPos, newRobotPos, numSteps);
    for step = 1:numSteps
        robot.model.animate(smoothTrajectory(step, :));

        % Collect path for Robot 1
        tr = robot.model.fkine(smoothTrajectory(step, :));
        pathRobot1 = [pathRobot1; tr.t'];

        % Move the apple with the robot
        delete(apple);
        apple = PlaceObject('apple5.ply', [tr.t(1), tr.t(2), tr.t(3)]);

        % Move the grippers with the robot
        finger1.model.base = tr.T * trotx(pi/2);
        finger2.model.base = tr.T * trotz(pi) * trotx(pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.animate(finger2.model.getpos());

        axis equal;
        drawnow();
    end
end
currentRobotPos = newRobotPos;

% Step 5: Lower the apple to the drop-off position
if ~stopFlag
    disp('Placing the apple at the drop-off position');
    targetPlace = transl(dropPosition(1), dropPosition(2), dropPosition(3) + safeZLimit) * trotx(pi);
    newRobotPos = robot.model.ikcon(targetPlace, currentRobotPos);
    smoothTrajectory = jtraj(currentRobotPos, newRobotPos, numSteps);
    for step = 1:numSteps
        robot.model.animate(smoothTrajectory(step, :));

        % Collect path for Robot 1
        tr = robot.model.fkine(smoothTrajectory(step, :));
        pathRobot1 = [pathRobot1; tr.t'];

        % Move the apple with the robot
        delete(apple);
        apple = PlaceObject('apple5.ply', [tr.t(1), tr.t(2), tr.t(3)]);

        % Move the grippers with the robot
        finger1.model.base = tr.T * trotx(pi/2);
        finger2.model.base = tr.T * trotz(pi) * trotx(pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.animate(finger2.model.getpos());

        axis equal;
        drawnow();
    end
end
currentRobotPos = newRobotPos;

% Step 5b: Drop the apple further to Z = 0.05
disp('Dropping apple to final height');
delete(apple);
apple = PlaceObject('apple5.ply', [dropPosition(1), dropPosition(2), 0.05]);
disp('Apple has been dropped to final position');

% Open the grippers after placing the apple
disp('Opening grippers to release the apple');
for step = 1:numSteps
    finger1.model.animate(qmato(step, :));
    finger2.model.animate(qmato(step, :));
    drawnow();
end

% Return robot 1 to its initial position
if ~stopFlag
    disp('Returning robot 1 to its starting position');
    smoothTrajectoryBack1 = jtraj(currentRobotPos, initialRobotPos, numSteps);
    for step = 1:numSteps
        robot.model.animate(smoothTrajectoryBack1(step, :));

        % Collect path for Robot 1
        tr = robot.model.fkine(smoothTrajectoryBack1(step, :));
        pathRobot1 = [pathRobot1; tr.t'];

        % Move the grippers with robot 1
        finger1.model.base = tr.T * trotx(pi/2);
        finger2.model.base = tr.T * trotz(pi) * trotx(pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.animate(finger2.model.getpos());

        axis equal;
        drawnow();
    end
end

% Path Plane Visualization for Robot 1
figure;
plot3(pathRobot1(:,1), pathRobot1(:,2), pathRobot1(:,3), 'b-', 'LineWidth', 1.5);
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
title('Path Plane of Robot 1');
grid on;
axis equal;

% Your initial setup remains unchanged up to where you start the main loop for r2.

%% Restock Process for r2 to pick up Apple6.ply and place it back at the initial pick position
% Define the positions where r2 will pick up and drop off the apple
pickPositionR2 = [0,1.85,0.1];  % Pick-up position for r2 (apple in Box1)
dropPositionR2 = [0.5, 0, 0.1];       % Drop-off position for r2 (previous pick-up location of robot1)

% Place the apple visually at the pick-up position for r2
apple2 = PlaceObject('apple5.ply', pickPositionR2);  % Place the apple at the new pick-up position

% Main Loop for r2: Move the apple from pick-up position to drop-off position

% Step 1: Move r2 above the pick-up position
if ~stopFlag
disp('r2: Moving to the apple at pick-up position');
targetAbovePickR2 = transl(pickPositionR2(1), pickPositionR2(2), pickPositionR2(3) + liftHeight) * trotx(pi);
newRobotPosR2 = r2.model.ikcon(targetAbovePickR2, initialRobotPosR2);
smoothTrajectoryR2 = jtraj(initialRobotPosR2, newRobotPosR2, numSteps);
for step = 1:numSteps
    r2.model.animate(smoothTrajectoryR2(step, :));
    
    % Move the grippers with r2
    tr = r2.model.fkine(smoothTrajectoryR2(step, :));
    finger3.model.base = tr.T * trotx(pi/2);
    finger4.model.base = tr.T * trotz(pi) * trotx(pi/2);
    finger3.model.animate(finger3.model.getpos());
    finger4.model.animate(finger4.model.getpos());

    axis equal;
    drawnow();
end
end
currentRobotPosR2 = newRobotPosR2;

% Step 2: Move down to pick up the apple for r2
if ~stopFlag

disp('r2: Moving down to pick up the apple');
targetPickR2 = transl(pickPositionR2(1), pickPositionR2(2), pickPositionR2(3) + safeZLimit) * trotx(pi);
newRobotPosR2 = r2.model.ikcon(targetPickR2, currentRobotPosR2);
smoothTrajectoryR2 = jtraj(currentRobotPosR2, newRobotPosR2, numSteps);
for step = 1:numSteps
    r2.model.animate(smoothTrajectoryR2(step, :));
    
    % Move the grippers with r2
    tr = r2.model.fkine(smoothTrajectoryR2(step, :));
    finger3.model.base = tr.T * trotx(pi/2);
    finger4.model.base = tr.T * trotz(pi) * trotx(pi/2);
    finger3.model.animate(finger3.model.getpos());
    finger4.model.animate(finger4.model.getpos());

    axis equal;
    drawnow();
end

currentRobotPosR2 = newRobotPosR2;

% Close the grippers for r2 to pick up the apple
disp('r2: Closing grippers to pick up the apple');
for step = 1:numSteps
    finger3.model.animate(qmatc(step, :));
    finger4.model.animate(qmatc(step, :));
    drawnow();
end
end
% Step 3: Lift the apple for r2 after picking it up
if ~stopFlag

disp('r2: Lifting the apple');
targetLiftR2 = transl(pickPositionR2(1), pickPositionR2(2), pickPositionR2(3) + liftHeight) * trotx(pi);
newRobotPosR2 = r2.model.ikcon(targetLiftR2, currentRobotPosR2);
smoothTrajectoryR2 = jtraj(currentRobotPosR2, newRobotPosR2, numSteps);
for step = 1:numSteps
    r2.model.animate(smoothTrajectoryR2(step, :));
    
    % Move the apple along with r2's end-effector
    applePosR2 = r2.model.fkine(smoothTrajectoryR2(step, :));
    delete(apple2);
    apple2 = PlaceObject('apple5.ply', [applePosR2.t(1), applePosR2.t(2), applePosR2.t(3)]);
    
    % Move the grippers with r2
    finger3.model.base = applePosR2.T * trotx(pi/2);
    finger4.model.base = applePosR2.T * trotz(pi) * trotx(pi/2);
    finger3.model.animate(finger3.model.getpos());
    finger4.model.animate(finger4.model.getpos());

    axis equal;
    drawnow();
end
end
currentRobotPosR2 = newRobotPosR2;

% Step 4: Move r2 to the drop-off position
if ~stopFlag

disp('r2: Moving to the drop-off position');
targetAbovePlaceR2 = transl(dropPositionR2(1), dropPositionR2(2), dropPositionR2(3) + liftHeight) * trotx(pi);
newRobotPosR2 = r2.model.ikcon(targetAbovePlaceR2, currentRobotPosR2);
smoothTrajectoryR2 = jtraj(currentRobotPosR2, newRobotPosR2, numSteps);
for step = 1:numSteps
    r2.model.animate(smoothTrajectoryR2(step, :));

    % Move the apple with r2
    applePosR2 = r2.model.fkine(smoothTrajectoryR2(step, :));
    delete(apple2);
    apple2 = PlaceObject('apple5.ply', [applePosR2.t(1), applePosR2.t(2), applePosR2.t(3)]);

    % Move the grippers with r2
    finger3.model.base = applePosR2.T * trotx(pi/2);
    finger4.model.base = applePosR2.T * trotz(pi) * trotx(pi/2);
    finger3.model.animate(finger3.model.getpos());
    finger4.model.animate(finger4.model.getpos());

    axis equal;
    drawnow();
end
end
currentRobotPosR2 = newRobotPosR2;

% Step 5: Lower the apple for r2 to the drop-off position
if ~stopFlag

disp('r2: Placing the apple at the drop-off position');
targetPlaceR2 = transl(dropPositionR2(1), dropPositionR2(2), dropPositionR2(3) + safeZLimit) * trotx(pi);
newRobotPosR2 = r2.model.ikcon(targetPlaceR2, currentRobotPosR2);
smoothTrajectoryR2 = jtraj(currentRobotPosR2, newRobotPosR2, numSteps);
for step = 1:numSteps
    r2.model.animate(smoothTrajectoryR2(step, :));

    % Move the apple along with r2's end-effector
    applePosR2 = r2.model.fkine(smoothTrajectoryR2(step, :));
    delete(apple2);
    apple2 = PlaceObject('apple5.ply', [applePosR2.t(1), applePosR2.t(2), applePosR2.t(3)]);

    % Move the grippers with r2
    finger3.model.base = applePosR2.T * trotx(pi/2);
    finger4.model.base = applePosR2.T * trotz(pi) * trotx(pi/2);
    finger3.model.animate(finger3.model.getpos());
    finger4.model.animate(finger4.model.getpos());

    axis equal;
    drawnow();
end

currentRobotPosR2 = newRobotPosR2;

% Step 5b: Drop the apple for r2 to Z = 0.05
disp('r2: Dropping apple to final height');
delete(apple2);
apple2 = PlaceObject('apple5.ply', [dropPositionR2(1), dropPositionR2(2), 0.05]);
disp('r2: Apple has been dropped to final position');

% Open the grippers for r2 after placing the apple
disp('r2: Opening grippers to release the apple');
for step = 1:numSteps
    finger3.model.animate(qmato(step, :));
    finger4.model.animate(qmato(step, :));
    drawnow();
end
end

% Return r2 to its initial position
if ~stopFlag

disp('r2: Returning to its starting position');
smoothTrajectoryBack2 = jtraj(currentRobotPosR2, initialRobotPosR2, numSteps);
for step = 1:numSteps
    r2.model.animate(smoothTrajectoryBack2(step, :));
    
    % Move the grippers with r2
    tr = r2.model.fkine(smoothTrajectoryBack2(step, :));
    finger3.model.base = tr.T * trotx(pi/2);
    finger4.model.base = tr.T * trotz(pi) * trotx(pi/2);
    finger3.model.animate(finger3.model.getpos());
    finger4.model.animate(finger4.model.getpos());

    axis equal;
    drawnow();
end
end

