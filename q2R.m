function R = q2R(q)
% Q2R
%   R = q2R(q)
%   
%   converts a quaternion into a 3 x 3 Rotation Matrix according to the
%   Euler-Rodrigues formula.  Expects quaternion as q = [q0;qv]
%
%   R = I + 2*q0*hat(qv) + 2*hat(qv)^2
    R = eye(3)+2*q(1)*hat(q(2:4))+2*hat(q(2:4))*hat(q(2:4));
end