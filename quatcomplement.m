function qc = quatcomplement(q)
    % QUATCOMPLEMENT
    %
    % qc = quatcomplement(q)
    %
    % generates the quaternion complement
    %
    % in: q = [q0;qv];
    % out: qc = [q0;-qv];
        
    qc = [q(1);-q(2:4)];
    
    
end