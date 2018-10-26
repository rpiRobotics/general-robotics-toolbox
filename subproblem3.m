function theta = subproblem3(p, q, k, d)
% SUBPROBLEM3
%   theta = subproblem3(p, q, k, d)
%   
%   solve for theta in an elbow joint according to
%   || q - rot(k, theta)*p || = d
%
%   input: p, q, k as R^3 column vectors, 
%        d as scalar
%   output: theta (2x1 vector, 0,1,2 solutions)
%
%   see SUBPROBLEM3

    pp = p - (k'*p)*k;
    qp = q - (k'*q)*k;
    dpsq = d^2 - (k'*(p+q))^2;

    bb = -(pp'*pp + qp'*qp - dpsq) / (2*norm(pp)*norm(qp));
    % cases with no solution
    if dpsq < 0 || abs(bb)>1 
        theta=[];
        return;
    end
    
    % nominal case
    theta = subproblem1(-pp/norm(pp), qp/norm(qp), k);
    % possible 2 solution case
    phi = acos(bb);
    if abs(phi) > 0
        theta = theta + phi*[1;-1];
    end
end
