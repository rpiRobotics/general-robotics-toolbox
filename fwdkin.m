
function [R,p]=fwdkin(robot,theta)
    % FWDKIN
    %
    % [R, p]=fwdkin(robot, theta)
    %
    % purpose: general forward kinematics for serial chain robot
    %           (compatible with matlab-rigid-body-viz toolbox)
    % 
    % input:
    %   robot: struct with form:
    %       root
    %           -> kin
    %               -> H          : [h_1 h_2 ... h_n]  
    %                               (3 x n) actuator axes
    %               -> P          : [p_{0,1} p_{1,2} ... p_{n-1,n}] 
    %                               (3 x n + 1) actuator displacements
    %               -> joint_type : n-vector of joint types
    %                               0 - rotational
    %                               1 - prismatic
    %                               2 - mobile orientation
    %                               3 - mobile translation
    %   
    %   theta: n-vector of actuator state (expecting radians for
    %                                           rotational states)
    % 
    % output:
    %       R = R_{0,n}: 3 x 3 matrix for orientation of end effector with
    %                   respect to base frame
    %       p = p_{0,n}: 3 x 1 vector giving position of end effector with
    %                   respect to base frame in base frame coordinates
    %

    if ~isfield(robot,'kin')
        error('fwdkin:fieldnotfound', ...
                'Could not find field kin in robot structure');
    end
    
    P=robot.kin.P;
    type=robot.kin.joint_type;
    H=robot.kin.H;
    
    p = P(:,1);
    R = eye(3);

    for i = 1:numel(type)
        if any(type(i) == [0 2])        % rotational actuators
            R = R*rot(H(:,i),theta(i));
        elseif any(type(i) == [1 3])    % translational actuators
            p = p + R*H(:,i)*theta(i);
        end
        p = p + R*P(:,i+1);
    end
end