function W_map = myWrenchMapCoM(ct,cf,n,L,CoM_Offset,CoM_Mode)
%MYWRENCHMAPCOM Wrench map 6x8 using rotor thrust directions directly.
%
%   W_map = myWrenchMapCoM(ct, cf, n, L, CoM_Offset)
%
% Inputs:
%   ct, cf : torque/force coefficients
%   n      : 3x8 matrix, each column is thrust direction (will be normalized)
%   L      : arm length (cube corner distance from CoM)
%
% Output:
%   W_map  : 6x8 wrench map [F; M]

    % spin direction coefficients 
    coef = [-1 1 -1 1 -1 1 -1 1];

    % rotor positions relative to CoM 
    P = [  L  -L  -L   L   L  -L  -L   L;
           L   L  -L  -L   L   L  -L  -L;
           L   L   L   L  -L  -L  -L  -L ];

    if CoM_Mode == 1 || CoM_Mode == 2
        P = P - CoM_Offset;
    end

    F = cf * n;     

    M = zeros(3,8);
    for i = 1:8
        z  = n(:,i);
        p  = P(:,i);
        ki = coef(i);

        M(:,i) = cf * cross(p, z) - ki * ct * z;
    end

    W_map = [F; M];
end
