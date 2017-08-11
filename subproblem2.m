function [theta1, theta2] = subproblem2(p, q, k1, k2)
% SUBPROBLEM2
%   [theta1, theta2] = subproblem2(p, q, k1, k2)
%
%   solve for two coincident, nonparallel axes rotating a link p, i.e.
%   q = rot(k1, theta1) * rot(k2, theta2) * p
%   solves by looking for intersections between cones of 
%               rot(k1,-theta1)q = rot(k2, theta2) * p
%       may have 0, 1, or 2 answers
%
%   input: p, q, k1, k2 as R^3 column vectors
%   output: theta1 and theta2 as 2x1 columns corresponding to the two
%           solutions, i.e. theta1 = [theta1 (solution 1); theta1 (solution 2)]
%
%   see also SUBPROBLEM1

    k12 = k1'*k2;
    pk = p'*k2;
    qk = q'*k1;

    % check if solutions exist
    if abs(1 - k12^2) < eps
        theta1=[];
        theta2=[];
        disp('no solution -  k1 =\= k2');
        return;
    end

    a = [k12 -1;-1 k12] * [pk;qk] / (k12^2 - 1);

    bb = (p'*p - a'*a - 2*a(1)*a(2)*k12);
    if abs(bb) < eps, bb=0;  end
    
    if bb<0
        theta1=[];
        theta2=[];
        disp('no solution - no intersection found between cones');
        return;
    end

    % check if there is only 1 solution
    gamma=sqrt(bb)/norm(cross(k1,k2));
    if abs(gamma)<eps;
        c1 = [k1 k2 cross(k1,k2)]*[a;gamma];
        theta2 = subproblem1(k2,p,c1);
        theta1 = -subproblem1(k1,q,c1); 
        return;
    end  

    % general case: 2 solutions
    c1 = [k1 k2 cross(k1,k2)]*[a;gamma];
    c2 = [k1 k2 cross(k1,k2)]*[a;-gamma];
    theta1 = -[subproblem1(q,c1,k1); subproblem1(q,c2,k1)];
    theta2 = [subproblem1(p,c1,k2); subproblem1(p,c2,k2)];

end
