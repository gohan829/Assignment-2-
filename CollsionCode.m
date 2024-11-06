clear all; % Clear all variables from the workspace
clc;       % Clear the command window (removes previous outputs)
hold on;   % Allow multiple plots to be drawn on the same figure
axis equal; % Make sure the axes are equally scaled so everything looks proportional

% Initialize the LinearUR3e robot model
robot = LinearUR3e(transl(0.6, -0.7, 0.05));

% Place tables and shelves in the environment
PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0.0, 0.0, -0.5]);
PlaceObject('tableBrown2.1x1.4x0.5m.ply', [-1.0, 0.0, -0.5]);
PlaceObject('tableBrown2.1x1.4x0.5m.ply', [-2.0, 0.0, -0.5]);

PlaceObject('shelf1.0.ply', [1, 0.3, 0]);
PlaceObject('shelf1.0.ply', [0, 0.3, 0]);
PlaceObject('shelf1.0.ply', [-1, 0.3, 0]);

% Define object bounding boxes for collision detection (xmin, xmax, ymin, ymax, zmin, zmax)
tableBoundingBoxes = [
    0, 2.1, -0.7, 0.7, -0.5, 0;   % Table at [0.0, 0.0, -0.5]
   -1, 1.1, -0.7, 0.7, -0.5, 0;   % Table at [-1.0, 0.0, -0.5]
   -2, 0.1, -0.7, 0.7, -0.5, 0;   % Table at [-2.0, 0.0, -0.5]
];

shelfBoundingBoxes = [
    0.5, 1.5, 0.1, 0.5, 0, 2.0;   % Shelf at [1, 0.3, 0]
   -0.5, 0.5, 0.1, 0.5, 0, 2.0;   % Shelf at [0, 0.3, 0]
   -1.5, -0.5, 0.1, 0.5, 0, 2.0;  % Shelf at [-1, 0.3, 0]
];

% Define a motion for the robot with collision detection
steps = 50; % Number of motion steps
currentPos = robot.model.getpos(); % Get the current joint positions
numJoints = length(currentPos); % Get the number of joints in the robot

for i = 1:steps
    % Generate a small random motion that matches the number of joints
    qNew = currentPos + (0.1 / steps) * rand(1, numJoints);
    
    % Update the robot's position with the new joint configuration
    robot.model.plot(qNew); % Use plot as an alternative to animate

    % Check for collisions
    isCollision = CheckCollision(robot, tableBoundingBoxes, shelfBoundingBoxes);
    if isCollision
        disp('Collision detected! Adjusting trajectory...');
        % Implement trajectory adjustment or stop motion here.
        break;
    end

    drawnow;
end

% Collision detection and helper functions (same as before)
function collision = CheckCollision(robot, tableBoundingBoxes, shelfBoundingBoxes)
    collision = false;
    linkPoints = GetRobotLinkPoints(robot);

    for i = 1:size(tableBoundingBoxes, 1)
        bounds = tableBoundingBoxes(i, :);
        if any(InBoundingBox(linkPoints, bounds))
            collision = true;
            return;
        end
    end

    for i = 1:size(shelfBoundingBoxes, 1)
        bounds = shelfBoundingBoxes(i, :);
        if any(InBoundingBox(linkPoints, bounds))
            collision = true;
            return;
        end
    end
end

function inside = InBoundingBox(points, bounds)
    inside = (points(:,1) >= bounds(1) & points(:,1) <= bounds(2)) & ...
             (points(:,2) >= bounds(3) & points(:,2) <= bounds(4)) & ...
             (points(:,3) >= bounds(5) & points(:,3) <= bounds(6));
end

function points = GetRobotLinkPoints(robot)
    points = [];
    for i = 1:length(robot.model.links)
        tr = robot.model.fkine(robot.model.getpos, i); 
        points = [points; tr(1:3, 4)']; 
    end
end
