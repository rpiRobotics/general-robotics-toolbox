function theta = subproblem1(p, q, k)
% SUBPROBLEM1
%   theta=subproblem1(p, q, k)
%
% solve for theta according to
%   q = rot(k, theta)*p
%
% input: k,p,q as R^3 column vectors
% output: theta (scalar)
%
% see SUBPROBLEM0
    
    % quickcheck for coincident vectors
    if norm(p-q) < sqrt(eps), theta=0; return; end
    
    % normalize axis k
    k = k / norm(k);
    % remove components parallel to k
    pp = p - (p'*k)*k;
    qp = q - (q'*k)*k;
    
    % normalize the vectors
    epp = pp / norm(pp);
    eqp = qp / norm(qp);

    % use subproblem0 to solve for theta
    theta = subproblem0(epp, eqp, k);

    if (abs(norm(p)-norm(q)) > 1e-2);
        disp('*** Warning *** ||p|| and ||q|| must be the same!!!');
    end
end