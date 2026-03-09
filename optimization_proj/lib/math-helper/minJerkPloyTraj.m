function [pdes, vdes, ades, tdes] = minJerkPloyTraj(wpts, vpts, apts, tpts, numsamples)
% Compute minimal jerk polynomial trajectory between position waypoints
% Inputs:
%   wpts       - 3 x N matrix of position waypoints
%   vpts       - 3 x N matrix of velocities at waypoints
%   apts       - 3 x N matrix of accelerations at waypoints
%   tpts       - 1 x N vector of time points for each waypoint
%   numsamples - Total number of samples in the trajectory
%
% Outputs:
%   pdes       - 3 x numsamples matrix of positions
%   vdes       - 3 x numsamples matrix of velocities
%   ades       - 3 x numsamples matrix of accelerations
%   tdes       - 1 x numsamples vector of time stamps

    % Ensure inputs are consistent
    assert(size(wpts,1) == 3, 'Waypoints must be 3xN');
    assert(isequal(size(wpts), size(vpts), size(apts)), 'Velocity and acceleration dimensions must match waypoints');
    assert(length(tpts) == size(wpts,2), 'Time vector length must match waypoint count');
    
    % Step 1: Compute durations and sample allocation
    durations = diff(tpts);
    duration_sum = sum(durations);
    segment_ratios = durations / duration_sum;
    segment_samples = round(numsamples * segment_ratios);
    
    % Step 2: Adjust rounding error
    segment_samples(end) = segment_samples(end) + (numsamples - sum(segment_samples));
    
    % Initialize outputs
    pdes = [];
    vdes = [];
    ades = [];
    tdes = [];
    
    % Loop over segments
    for i = 1:numel(tpts)-1
        T = tpts(i+1) - tpts(i);
        N = segment_samples(i);
    
        if i < numel(tpts)-1
            t = linspace(0, T, N+1);
            t = t(1:end-1); % avoid overlapping with next segment
        else
            t = linspace(0, T, N);
        end
        t_global = t + tpts(i);
    
        % Quintic coefficients matrix
        coefs_pos_mat = 1/T^5 * [720,    -350*T,   60*T^2;
                                -360*T,  168*T^2, -24*T^3;
                                 60*T^2, -24*T^3,  3*T^4];
    
        % Delta values
        del_p = wpts(:,i+1) - wpts(:,i) - vpts(:,i)*T - 0.5*T^2*apts(:,i);
        del_v = vpts(:,i+1) - vpts(:,i) - apts(:,i)*T;
        del_a = apts(:,i+1) - apts(:,i);
    
        % Allocate for this segment
        x   = zeros(3, length(t));
        dx  = zeros(3, length(t));
        ddx = zeros(3, length(t));
    
        % Compute trajectory for each axis
        for j = 1:3
            co_p = coefs_pos_mat * [del_p(j); del_v(j); del_a(j)];
            c = [co_p; apts(j,i); vpts(j,i); wpts(j,i)];
    
            x(j,:)   = c(1)/120 * t.^5 + c(2)/24 * t.^4 + c(3)/6 * t.^3 + c(4)/2 * t.^2 + c(5) * t + c(6);
            dx(j,:)  = c(1)/24 * t.^4 + c(2)/6 * t.^3 + c(3)/2 * t.^2 + c(4) * t + c(5);
            ddx(j,:) = c(1)/6 * t.^3 + c(2)/2 * t.^2 + c(3) * t + c(4);
        end
    
        % Append to full trajectory
        pdes = [pdes, x];
        vdes = [vdes, dx];
        ades = [ades, ddx];
        tdes = [tdes, t_global];
    end
    
end