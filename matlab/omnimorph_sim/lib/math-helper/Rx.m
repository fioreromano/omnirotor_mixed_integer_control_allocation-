function R = Rx(th)
%RX Elementary rotation about x-axis
    R = [1 0 0; 0 cos(th) -sin(th);0 sin(th) cos(th)];
end

