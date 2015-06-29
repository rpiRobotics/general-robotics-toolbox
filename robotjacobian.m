function J = robotjacobian(kin, theta)
    % ROBOTJACOBIAN
    %
    % J = robotjacobian(kin, theta)
    %
    % purpose: calculate jacobian for serial chain robot relating joint
    %           velocity to end effector spatial velocity
    %           (compatible with matlab-rigid-body-viz toolbox)
    %
    % input:
    %   kin: struct with form:
    %       root
    %           -> H          : [h_1 h_2 ... h_n]  
    %                           (3 x n) actuator axes
    %           -> P          : [p_{O,1} p_{1,2} ... p_{n,T}] 
    %                           (3 x n + 1) actuator displacements
    %           -> joint_type : n-vector of joint types
    %                           0 - rotational
    %                           1 - prismatic
    %                           2 - mobile orientation
    %                           3 - mobile translation
    %   
    %   theta: n-vector of actuator position (expecting radians for
    %                                           rotational states)
    % 
    % output:
    %       J = [jw_1 jw_2 ... jw_n] (6 x n) jacobian matrix 
    %           [jv_1 jv_2 ... jv_n]        * in base frame
    %           top three rows reflect the instantaneous change in 
    %           end effector angular velocity while the bottom three rows
    %           are the instantaneous change in linear velocity both with
    %           respect to actuator velocity
    %
    
    p = kin.P(:,1);
    R = eye(3);
    
    J = zeros(6,numel(kin.joint_type));

    hi = zeros(3,n);
    pOi = zeros(3,n+1);
    pOi(:,1) = p;

    % Compute and store forward kinematics
    for i = 1:n
        if (kin.joint_type(i) == 0 || ...       % rotational actuators
                    kin.joint_type(i) == 2)        
            R = R*rot(kin.H(:,i),theta(i));
        elseif (kin.joint_type(i) == 1 || ...   % translational actuators
                    kin.joint_type(i) == 3) 
            p = p + R*kin.H(:,i)*theta(i);
        end
        p = p + R*kin.P(:,i+1);
        pOi(:,i+1) = p;
        hi(:,i) = R*kin.H(:,i);
    end

    p0T = pOi(:,end);
    % Compute Jacobian
    for i=1:n
        if (kin.joint_type(i) == 0 || ...       % rotational actuators
                kin.joint_type(i) == 2)        
            J(:,i)=[hi(:,i); hat(hi(:,i))*(p0T - pOi(:,i))];
        elseif (kin.joint_type(i) == 1 || ...   % translational actuators
                kin.joint_type(i) == 3)   
            J(:,i)=[0;0;0; hi(:,i)];
        end
    end
end
   
   
    
    
