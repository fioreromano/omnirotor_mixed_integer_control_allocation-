function [u, v] = tangentBasis(n0)

    tmp = [1; 0; 0];
    if abs(dot(tmp, n0)) > 0.9
        tmp = [0; 1; 0];
    end

    u = cross(n0, tmp);
    u = u / norm(u);

    v = cross(n0, u);
    v = v / norm(v);

end
