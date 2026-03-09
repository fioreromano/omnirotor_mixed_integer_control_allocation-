function profile = smooth_profile(t, tfinal, final_angle,initial)
    % Compute coefficients for quintic polynomial
    a0 = initial;
    a1 = 0;
    a2 = 0;
    a3 = (10 * (final_angle-initial)) / tfinal^3;%10
    a4 = -(15 * (final_angle-initial)) / tfinal^4; %-15
    a5 = (6 * (final_angle-initial)) / tfinal^5;  %6
    
    % Initialize profile
    profile = zeros(size(t));
    
    % Generate smooth profile
    for i = 1:length(t)
        if t(i) <= tfinal
            profile(i) = a0 + a1 * t(i) + a2 * t(i)^2 + a3 * t(i)^3 + a4 * t(i)^4 + a5 * t(i)^5;
        else
            profile(i) = final_angle; % Ensure the final angle is reached
        end
    end
      % Compute velocity profile
    v = gradient(profile, t);
    
    % Compute acceleration profile
    a = gradient(v, t);
    
    % Clip velocity profile to enforce maximum velocity constraint
    v = min(max(v, -5), 5);
    
    % Clip acceleration profile to enforce maximum acceleration constraint
    a = min(max(a, -2.5), 2.5);
    
    % Integrate velocity profile to ensure consistency
    profile = cumtrapz(t, v)+initial;
end