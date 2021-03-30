function [rotated_point] = rotate_point(point, q)
% rotate a 3D point about a normalized quaternion

p = [0;point(1);point(2);point(3)];
q = q/norm(q,2);
q_conj = [q(1);-q(2);-q(3);-q(4)];

p_tmp = quat_mult(p,q_conj);
p_rotated = quat_mult(q, p_tmp);

rotated_point = [p_rotated(2);p_rotated(3);p_rotated(4)];