function G = screw_matrix(r)
    % Returns the matrix G representing the 6 x 6 transformation between
    % screws in a rigid body, i.e.
    % 
    %  (twist)_2 = transpose(G)*(twist)_1
    %           (twist) = [angular velocity, linear velocity]
    %  (wrench)_1 = G*(wrench)_2   
    %           (wrench) = [torque, force]
    %
    % r is the 3 x 1 displacement vector from point 1 to point 2
    
    G = eye(6);
    G(1:3,4:6) = hat(r);
    
end