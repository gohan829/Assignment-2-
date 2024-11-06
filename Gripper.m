classdef Gripper < RobotBaseClass
%% RobotiQ 2f-140-gripper
    properties (Access = public)
        plyFileNameStem = 'Gripper';
    end

    methods
%% Define robot Function 
        function self = Gripper(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = baseTr;
            
            self.PlotAndColourRobot();
        end
%% Create the robot model
        function CreateModel(self)
            link(1) = Link('d',0, 'a',0.05, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',-deg2rad(16));
            link(2) = Link('d',0, 'a',0.045, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',deg2rad(58));
            link(3) = Link('d',0, 'a',0.045, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',deg2rad(48));
          

            self.model = SerialLink(link, 'name', self.name);
        end
    end
end

