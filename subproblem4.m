function [theta]=subproblem4(p, q, k, d)
% SUBPROBLEM4
%   theta = subproblem4(p, q, k, d)
%   
%   solve for theta when static displacement from rotation axis from
%   d=p'*rot(k,theta)*q
%
%   input: p, q as R^3 vectors, k is a unit vector, d: scalar
%   output: theta (up to 2 solutions)

    c = d - (p'*q + p'*hat(k)*hat(k)*q);
    a = p'*hat(k)*q;
    b = -p'*hat(k)*hat(k)*q;

    phi = atan2(b,a);

    if abs(c / sqrt(a^2 + b^2)) > 1
        theta=[];
        return;
    end

    psi = asin(c / sqrt(a^2 + b^2));

    theta = -phi + [psi;-psi + pi];
end
