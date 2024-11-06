classdef CollisionModule
    properties
        robot  % Reference to the primary robot
        environmentObjects  % Cell array of objects in the environment
    end

    methods
        function obj = CollisionModule(robot, environmentObjects)
            obj.robot = robot;
            obj.environmentObjects = environmentObjects;
        end

        function collisionFlag = checkForCollision(obj, position)
            collisionFlag = false;
            robotPosition = obj.robot.model.fkine(position);

            for i = 1:length(obj.environmentObjects)
                objPos = getTransform(obj.environmentObjects{i});
                distance = norm(robotPosition(1:3, 4) - objPos(1:3, 4));

                if distance < 0.1
                    collisionFlag = true;
                    disp(['Collision detected with object: ', obj.environmentObjects{i}.Name]);
                    break;
                end
            end
        end

        function avoidCollision(obj, currentTrajectory)
            for step = 1:size(currentTrajectory, 1)
                if obj.checkForCollision(currentTrajectory(step, :))
                    disp('Avoiding collision, rerouting...');
                    newTrajectory = jtraj(currentTrajectory(step, :), currentTrajectory(end, :), 20);
                    for j = 1:size(newTrajectory, 1)
                        obj.robot.model.animate(newTrajectory(j, :));
                        drawnow();
                    end
                    break;
                end
            end
        end
    end
end
