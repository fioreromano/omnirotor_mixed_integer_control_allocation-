function [U,V,rhat] = buildPerpBases(p, eps_norm)
% pi_cg: 3xn, vettori CG->prop (fissi)
n = size(p,2);
rhat = p ./ max(vecnorm(p,2,1), eps_norm);

U = zeros(3,n);
V = zeros(3,n);

for j = 1:n
    ref = [0;0;1];
    if abs(dot(ref, rhat(:,j))) > 0.95
        ref = [1;0;0];
    end

    uj = cross(rhat(:,j), ref);
    uj = uj / max(norm(uj), eps_norm);

    vj = cross(rhat(:,j), uj);
    vj = vj / max(norm(vj), eps_norm);

    U(:,j) = uj;
    V(:,j) = vj;
end
end