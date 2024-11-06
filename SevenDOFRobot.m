classdef SevenDOFRobot
    properties
        model
    end
    
    methods
        function obj = SevenDOFRobot(baseTransform)
            if nargin < 1
                baseTransform = eye(4);
            end
            
            % Define a small scale factor
            scale = 0.025;  % You can adjust this value further if needed
            
            % Define DH parameters with the scale directly applied
            L1 = Link('d', 16*scale, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi/2, pi/9]);
            L2 = Link('d', 0, 'a', 15*scale, 'alpha', pi/2, 'qlim', [-pi/8, pi/3]);
            L3 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/6, pi/2]);
            L4 = Link('d', 0, 'a', 10*scale, 'alpha', pi/2, 'qlim', [-pi/9, pi/2]);
            L5 = Link('d', 0.3*scale, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/6, pi/2]);
            L6 = Link('d', 0, 'a', 5*scale, 'alpha', 0, 'qlim', [-pi, pi]);
            
            % Create the SerialLink model directly in the constructor
            obj.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '7DOFRobot');
            obj.model.base = baseTransform;  % Set the base position
        end
        
        function plotRobot(obj, q)
            if nargin < 2
                q = zeros(1, obj.model.n);  % Default configuration
            end
            obj.model.plot(q);  % Plot the robot in the given configuration
        end

        function moveArm(self, destination, steps)
            if nargin < 3
                steps = 75;
            end
            qCurrent = self.model.getpos();
            qFinal = self.model.ikcon(destination);           
            qMatrix = jtraj(qCurrent, qFinal, steps);
            for i = 1:steps
                self.model.animate(qMatrix(i, :));
                drawnow();
            end
        end
    end
end
