function Q = quatproduct(q)
    % QUATPRODUCT
    %
    % Q = quatproduct(q)
    %
    % generates matrix representation of a Hamilton quaternion product
    % operator
    %
    % in: q = [q0;qv];
    % out: Q = [q0 -qv'; qv q0*eye(3)+ cross(qv)]
        
    Q = [q(1) -q(2:4)'; q(2:4) q(1)*eye(3)+hat(q(2:4))];
    
    
end