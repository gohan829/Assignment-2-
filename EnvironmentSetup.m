classdef EnvironmentSetup
    properties
        Table
        Concrete
        
    end
    
    methods
        %% Section With the enivourmental

       function obj = EnvironmentSetup()
    % Define new smaller coordinates for barriers to form a fence closer to the robot
    % Shorten the barriers to fit better around the robot
  axis([-2.5 2.5 -2.5 2.5 -2.5 2.5])
view(3)

    % Place emergency button and fire extinguishers attached to the walls
   PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0.0, 0.0, -0.5]);  % Place lower
   hold on;
  
   PlaceObject('tableBrown2.1x1.4x0.5m.ply', [-1.0, 0.0, -0.5]);  % Place lower
   hold on;
  
   PlaceObject('tableBrown2.1x1.4x0.5m.ply', [-2.0, 0.0, -0.5]);  % Place lower
   hold on;
   

   
   PlaceObject('shelf1.0.ply', [1, 0.3, 0] );  % Place lower 
      hold on;
   
    PlaceObject('shelf1.0.ply', [0, 0.3, 0] );  % Place lower  
     hold on;
    
    PlaceObject('shelf1.0.ply', [-1, 0.3, 0] );  % Place lower  
    hold on;
   
     

      PlaceObject('SHOPPP.ply', [0.8, -1.2, -0.5] );  % Place lower  
    hold on; 
     PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0,-2.5,-0.5]);  % Place lower
   hold on;
    % Plot the concrete foundation just under the robot within the fenced area
   
end
    end
end
