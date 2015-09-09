function q = R2q(R)
    % R2Q
    %   
    %   q = R2q(R)
    %   
    %   Converts a 3 x 3 Rotation Matrix into a quaternion.  Quaternion is
    %   returned in the form q = [q0;qv].  

    tr = trace(R);
    q = zeros(4,1);
    % Need to handle cases of poor numerical scaling
    if tr > 0
        S = 2*sqrt(tr+1);
        q(1) = 0.25*S;
        q(2) = (R(3,2) - R(2,3)) / S;
        q(3) = (R(1,3) - R(3,1)) / S;
        q(4) = (R(2,1) - R(1,2)) / S;
    elseif R(1,1)>R(2,2) && R(1,1)> R(3,3)
        S = 2*sqrt(1 + R(1,1) - R(2,2) - R(3,3));
        q(1) = (R(3,2) - R(2,3)) / S;
        q(2) = 0.25*S;
        q(3) = (R(1,2) + R(2,1)) / S;
        q(4) = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = 2*sqrt(1 + R(2,2) - R(1,1) - R(3,3));
        q(1) = (R(1,3) - R(3,1)) / S;
        q(2) = (R(1,2) + R(2,1)) / S;
        q(3) = 0.25*S;
        q(4) = (R(2,3) + R(3,2)) / S;
    else
        S = 2*sqrt(1 + R(3,3) - R(1,1) - R(2,2));
        q(1) = (R(2,1) - R(1,2)) / S;
        q(2) = (R(1,3) + R(3,1)) / S;
        q(3) = (R(2,3) + R(3,2)) / S;
        q(4) = 0.25*S;
    end
end