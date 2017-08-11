function theta = subproblem0(p, q, k)
% SUBPROBLEM0
%   theta = subproblem0(p, q, k)
%
%   solve for theta subtended between p and q according to
%           q = rot(k, theta)*p
%           ** assumes k'*p = 0 and k'*q = 0
%
%   input: p,q,k as R^3 column vectors
%   output: theta (scalar)
%   
%   see SUBPROBLEM1

    if k'*p > sqrt(eps) || k'*q > sqrt(eps)
        error('k must be perpendicular to p and q');
    end

    ep=p/norm(p);
    eq=q/norm(q);

    theta=2*atan2(norm(ep-eq),norm(ep+eq));

    if k'*(cross(p,q))<0
      theta=-theta;
    end
end
