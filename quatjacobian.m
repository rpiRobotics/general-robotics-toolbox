function Jq = quatjacobian(q)
    % QUATJACOBIAN
    %
    % Jq = quatjacobian(q)
    %
    % in: (4 x 1) unit quaternion q = [q0;qv]
    %
    % out: (4 x 3) Jacobian Jq that relates (3 x 1) angular velocity 
    %               to (4 x 1) unit quaternion velocity
    
    q0 = q(1);
    qv = q(2:4);
    
    Jq = 1/2*[-qv';q0*eye(3)-hat(qv)];
end