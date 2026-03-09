function [R_out, omega_out, domega_out, t_out] = minAaccPolyTraj(rots, ompts, tpts, numsamples)
% Compute minimal angular acceleration polynomial trajectory between
% orientation waypoints
% Inputs:
%   rots       - 3x3xN rotation matrices (SO(3))
%   ompts      - 3xN angular velocities at keyframes (in body frame)
%   tpts       - 1xN time points
%   numsamples - total number of samples desired
%
% Outputs:
%   R_out      - 3x3xnumsamples interpolated rotation matrices
%   omega_out  - 3xnumsamples angular velocities (in world frame)
%   domega_out  -3xnumsamples angular acceleration (in world frame)
%   t_out      - 1xnumsamples time values

    % Ensure dimensions match
    assert(size(rots,3) == size(ompts,2), 'Mismatch in number of rotation frames and omegas.');
    assert(numel(tpts) == size(rots,3), 'tpts must match number of rotation matrices.');
    
    % Precompute exponential coordinate differences and blended angular velocities
    numSegments = numel(tpts) - 1;
    rpts = zeros(3, numSegments);
    omTild = zeros(3, numSegments);
    
    for l = 1:numSegments
        RotRel = rots(:,:,l)' * rots(:,:,l+1);
        re_skew = logm(RotRel);
        r_e = [re_skew(3,2); re_skew(1,3); re_skew(2,1)];
        omegaTilde = WMat(r_e) \ ompts(:,l+1); 
    
        rpts(:,l) = r_e;
        omTild(:,l) = omegaTilde;
    end
    
    % Compute sample distribution
    durations = diff(tpts);
    totalTime = sum(durations);
    segmentSamples = round(numsamples * durations / totalTime);
    segmentSamples(end) = segmentSamples(end) + (numsamples - sum(segmentSamples)); % fix rounding
    
    % Initialize outputs
    R_out = zeros(3, 3, numsamples);
    omega_out = zeros(3, numsamples);
    t_out = zeros(1, numsamples);
    
    idx = 1;
    
    for i = 1:numSegments
        T = durations(i);
        N = segmentSamples(i);
        
        if i < numSegments
            tLocal = linspace(0, T, N+1);
            tLocal = tLocal(1:end-1);  % avoid overlap
        else
            tLocal = linspace(0, T, N);
        end
        tGlobal = tLocal + tpts(i);
    
        % Polynomial coefficients
        coefsRotMat = 1/T^3 * [-12, 6*T; 6*T, -2*T^2];
    
        r = zeros(3, length(tLocal));
        dr = zeros(3, length(tLocal));
    
        for j = 1:3
            co = coefsRotMat * [rpts(j,i) - ompts(j,i)*T; omTild(j,i) - ompts(j,i)];
            d = [co; ompts(j,i)];
    
            r(j,:) = d(1)/6 * tLocal.^3 + d(2)/2 * tLocal.^2 + d(3) * tLocal;
            dr(j,:) = d(1)/2 * tLocal.^2 + d(2) * tLocal + d(3);
        end
    
        % Interpolate rotations and compute angular velocity
        for k = 1:length(tLocal)
            RotInterp = rots(:,:,i) * expm(skewMat(r(:,k)));
            omegaInterp = RotInterp * WMat(r(:,k)) * dr(:,k);  % world frame angular velocity
    
            R_out(:,:,idx) = RotInterp;
            omega_out(:,idx) = omegaInterp;
            t_out(idx) = tGlobal(k);
            idx = idx + 1;
        end
        
    end
    domega_out = zeros(3, numsamples);
    for i = 1:3  % for each component x, y, z
        % Central difference for interior points
        domega_out(i,2:end-1) = (omega_out(i,3:end) - omega_out(i,1:end-2)) ./ (t_out(3:end) - t_out(1:end-2));
        
        % Forward difference for first point
        domega_out(i,1) = (omega_out(i,2) - omega_out(i,1)) / (t_out(2) - t_out(1));
        
        % Backward difference for last point
        domega_out(i,end) = (omega_out(i,end) - omega_out(i,end-1)) / (t_out(end) - t_out(end-1));
    end

end