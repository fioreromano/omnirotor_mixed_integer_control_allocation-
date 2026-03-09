function [q_out,qdt_out,qddt_out] = circ_traj(t,r,f,z)

radius = r;
w = 2*pi*f;

q_out = zeros(3,1);
qdt_out = zeros(3,1);
qddt_out = zeros(3,1);

q_out(1) = radius*cos(w*t);
q_out(2) = radius*sin(w*t);
q_out(3) = z;

qdt_out(1) = -w*radius*sin(w*t);
qdt_out(2) = w*radius*cos(w*t);
qdt_out(3) = 0;

qddt_out(1) = -w^2 *radius*cos(w*t);
qddt_out(2) = -w^2 *radius*sin(w*t);
qddt_out(3) = 0;

end