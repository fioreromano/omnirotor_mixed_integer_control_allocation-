function W_map = myWrenchMap(ct,cf,al,L)
%MYWRENCHMAP Constructs the wrench map matrix for an 8-motor configuration.
%
%   W_map = myWrenchMap(ct, cf, al, L)
%
%   This function computes the 6×8 wrench map matrix W_map that relates
%   individual rotor thrusts to the total wrench (force and torque) 
%   exerted on the body frame.
%
%   Inputs:
%       ct  - Torque coefficient (relates thrust to motor reaction torque)
%       cf  - Force coefficient (scaling factor for thrust magnitude)
%       al  - Tilt angle of each rotor from vertical (in radians)
%       L   - Distance from the center to each motor (arm length)
%
%   Output:
%       W_map - 6×8 matrix where each column corresponds to the force (top 3 rows)
%               and torque (bottom 3 rows) contribution of a single rotor.
%
%   The configuration consists of 8 tilted rotors placed at the corners of a cube.
%   Each rotor generates a thrust vector (z_i) tilted by angle al, and a torque 
%   that includes both the moment arm (from position) and motor-induced torque
%   (scaled by ct). The directions of spin (used for torque sign) are encoded in `coef`.

    % Rotor spin direction coefficients (for reaction torque)
    coef = [-1 1 -1 1 -1 1 -1 1];
    
    % Compute forces and torques for each rotor
    p1 = [L;L;L];
    z_p1 = [-sin(al);0;cos(al)];
    ki = coef(1);
    f1 = cf*z_p1;
    m1 = cf*cross(p1,z_p1) - ki*ct*z_p1;
    
    p2 = [-L;L;L];
    z_p2 = [0;-sin(al);cos(al)];
    ki = coef(2); 
    f2 = cf*z_p2; 
    m2 = cf*cross(p2,z_p2) - ki*ct*z_p2;
    
    p3 = [-L;-L;L];
    z_p3 = [sin(al);0;cos(al)];
    ki = coef(3);
    f3 = cf*z_p3; 
    m3 = cf*cross(p3,z_p3) - ki*ct*z_p3;
    
    p4 = [L;-L;L];
    z_p4 = [0;sin(al);cos(al)];
    ki = coef(4);
    f4 = cf*z_p4; 
    m4 = cf*cross(p4,z_p4) - ki*ct*z_p4;
    
    p5 = [L;L;-L];
    z_p5 = [sin(al);0;cos(al)];
    ki = coef(5); 
    f5 = cf*z_p5; 
    m5 = cf*cross(p5,z_p5) - ki*ct*z_p5;
    
    p6 = [-L;L;-L];
    z_p6 = [0;sin(al);cos(al)];
    ki = coef(6);
    f6 = cf*z_p6; 
    m6 = cf*cross(p6,z_p6) - ki*ct*z_p6;
    
    p7 = [-L;-L;-L];
    z_p7 = [-sin(al);0;cos(al)];
    ki = coef(7);
    f7 = cf*z_p7; 
    m7 = cf*cross(p7,z_p7) - ki*ct*z_p7;
    
    p8 = [L;-L;-L];
    z_p8 = [0;-sin(al);cos(al)];
    ki = coef(8);
    f8 = cf*z_p8;
    m8 = cf*cross(p8,z_p8) - ki*ct*z_p8;
    
    % Assemble wrench map: top rows = forces, bottom rows = torques
    W_map = [f1,f2,f3,f4,f5,f6,f7,f8;
             m1,m2,m3,m4,m5,m6,m7,m8];

end


