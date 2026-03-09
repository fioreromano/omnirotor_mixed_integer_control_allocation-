function [x, y] = uniform_xy_on_upper_s2(M)
    % Generate M points uniformly on the upper hemisphere z>=0

    u = rand(M,1);   % uniform in [0,1]
    v = rand(M,1);   % uniform in [0,1]

    z = u;                   % z in [0,1] directly gives uniform hemisphere
    phi = 2*pi*v;            % azimuth uniform
    
    r = sqrt(1 - z.^2);      % radial component in xy-plane

    x = r .* cos(phi);
    y = r .* sin(phi);
end
