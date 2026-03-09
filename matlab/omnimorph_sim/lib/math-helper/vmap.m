function eR = vmap(Rd,R)
% Orientation error between rotation matrices
    E = 0.5 * ((R'*Rd) - (Rd'*R));
 
    eR =[(E(3, 2) - E(2, 3))/2;...
         (E(1, 3) - E(3, 1))/2;...
         (E(2, 1) - E(1, 2))/2];
    
end

