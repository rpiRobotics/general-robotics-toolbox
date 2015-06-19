function J=robotjacobian(robot,theta)
    % ROBOTJACOBIAN
    %
    % J = robotjacobian(robot, theta)
    %
    % purpose: calculate jacobian for serial chain robot relating joint
    %           velocity to end effector spatial velocity
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
    %   theta: n-vector of actuator position (expecting radians for
    %                                           rotational states)
    % 
    % output:
    %       J = [jw_1 jw_2 ... jw_n] (6 x n) jacobian matrix according to
    %           [jv_1 jv_2 ... jv_n]        [d-w/d-theta; d-v/d-theta]
    %                                       
    %
    
    if ~isfield(robot,'kin')
        error('robotjacobian:fieldnotfound', ...
                'Could not find field kin in robot structure');
    end
    
    P=robot.kin.P;
    type=robot.kin.type;
    H=robot.kin.H;
    
    p = P(:,1);
    R = eye(3);
    
    J=zeros(6,numel(type));

    hi = zeros(3,n);
    pOi = zeros(3,n+1);
    pOi(:,1) = p;

    % Store forward kinematics
    for i = 1:n
        if any(type(i) == [0 2])        % rotational actuators
            R = R*rot(H(:,i),theta(i));
            p = p + R*P(:,i+1);
        elseif any(type(i) == [1 3])    % translational actuators
            p = p + R*(H(:,i)*theta(i) + P(:,i+1));
        end
        pOi(:,i+1) = p;
        hi(:,i) = R*H(:,i);
    end

    p0T = pOi(:,end);
    % Compute Jacobian
    for i=1:n
        if any(type(i) == [0 2])        % rotational actuators
            J(:,i)=[hi(:,i); hat(hi(:,i))*(p0T - pOi(:,i))];
        elseif any(type(i) == [1 3])    % translational actuators
            J(:,i)=[0;0;0; hi(:,i)];
        end
    end
end
   
   
    
    
