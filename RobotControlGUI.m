function RobotControlGUI(robot, finger1, finger2, qmatc, qmato, initialRobotPos)
    % Create the GUI figure
    fig = uifigure('Name', 'Clicky', 'Position', [100, 100, 400, 650]);

    % Global stop flag and initial state
    global stopFlag currentRobotPos startMainLoop;
    stopFlag = false;  
    currentRobotPos = initialRobotPos;
    startMainLoop = false;

    % Add a label for joint sliders
    uilabel(fig, 'Position', [20, 600, 360, 20], 'Text', 'Joint Controls');

    % Add sliders for each joint angle with adjusted spacing
    numJoints = length(initialRobotPos);
    jointSliders = gobjects(1, numJoints);  % Adjust size to number of joints in robot
    sliderStartY = 560;  % Initial Y-position for the first slider
    sliderSpacing = 60;  % Vertical spacing between sliders

    for i = 1:numJoints
        yPos = sliderStartY - (i - 1) * sliderSpacing;
        jointSliders(i) = uislider(fig, 'Position', [20, yPos, 360, 3], ...
            'Limits', [-pi, pi], 'ValueChangedFcn', @(sld,event) updateRobotJoint(sld, i));
        % Adjusted label position to be lower
        uilabel(fig, 'Position', [20, yPos - 25, 360, 20], 'Text', ['Joint ' num2str(i)]);
    end

    % Gripper control buttons
    gripOpenBtn = uibutton(fig, 'Text', 'Open Gripper', 'Position', [20, 50, 160, 30], ...
        'ButtonPushedFcn', @(btn,event) controlGripper('open'));
    gripCloseBtn = uibutton(fig, 'Text', 'Close Gripper', 'Position', [200, 50, 160, 30], ...
        'ButtonPushedFcn', @(btn,event) controlGripper('close'));

    % Reset and Emergency Stop buttons
    resetBtn = uibutton(fig, 'Text', 'Reset', 'Position', [20, 10, 160, 30], ...
        'ButtonPushedFcn', @(btn,event) resetSimulation());
    stopBtn = uibutton(fig, 'Text', 'Emergency Stop', 'Position', [200, 10, 160, 30], ...
        'ButtonPushedFcn', @(btn,event) emergencyStop());

    % Resume button
    resumeBtn = uibutton(fig, 'Text', 'Resume', 'Position', [200, 90 , 160, 30], ...
        'ButtonPushedFcn', @(btn,event) resumeOperation());

    %% Nested Functions

    % Function to update robot joints based on slider values
    function updateRobotJoint(sld, jointIdx)
        currentRobotPos = robot.model.getpos();
        currentRobotPos(jointIdx) = sld.Value;
        robot.model.animate(currentRobotPos);
        drawnow();
    end

    % Function to control gripper
    function controlGripper(action)
        if strcmp(action, 'close')
            animateGripper(qmatc);
        elseif strcmp(action, 'open')
            animateGripper(qmato);
        end
    end

    % Function to animate gripper open/close
    function animateGripper(trajectory)
        for step = 1:length(trajectory)
            while stopFlag, pause(0.1); end  % Pauses if stopFlag is true
            finger1.model.animate(trajectory(step, :));
            finger2.model.animate(trajectory(step, :));
            drawnow();
        end
    end

    % Function for emergency stop
    function emergencyStop()
        stopFlag = true;
        disp('Emergency Stop Activated');
    end
end
