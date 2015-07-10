function J = unicyclejacobian(kin, theta)
    % UNICYCLEJACOBIAN
    %
    % J = unicyclejacobian(kin, theta)
    %
    % expecting kin to have format
    %   kin.H = actuator axes
    %   kin.P = inter-actuator displacements
    %   kin.joint_type = actuator types
    %           (looking for [3 3 2]) block for unicycle jacobian)
    %
    % theta is a vector containing
    %   [position 1, position 2, orientation]
    %       position 1 is the cartesian location of the unicycle in the
    %           world w.r.t. its wheel orientation (i.e. if the robot can
    %           travel along its body x-axis, position 1 is the cartesian
    %           location of the robot in the world x-axis)
    %       position 2 is the cartesian location of the unicycle in the
    %           world in the orthogonal direction to its wheels 
    %           (i.e. if the robot can travel along its body x-axis, 
    %           position 2 is the cartesian location of the robot in the 
    %           world y-axis)
    %       orientation is the angle about its center of mass
    %
    %             [jp1_v 0] (map to world position 1 velocity)
    % returns J = [jp2_v 0] (map to world position 2 velocity)
    %             [0     1] (map to world orientation velocity)
    %
    %           jacobian that maps unicycle velocity [v;w] to cartesian
    %           velocity
    
    idx = strfind(kin.joint_type,[3 3 2]);
    if isempty(idx)
        error('unicyclejacobian:no_mobile_block', ...
                'Could not find unicycle block in provided kinematics');
    end
    
    J = [cos(theta(3)) 0;sin(theta(3)) 0;0 1];
    
end