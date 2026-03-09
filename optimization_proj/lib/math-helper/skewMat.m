function v_skwmat = skewMat(v)
% SKEWMAT Computes the skew-symmetric matrix associated to vector v 
%(belonging to R3).

    v_skwmat = [  0   -v(3)  v(2);...
                v(3)  0    -v(1);...
                -v(2)  v(1)   0   ];
    
end

