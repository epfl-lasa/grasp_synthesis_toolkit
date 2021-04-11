function [rotated_point] = rotate_point(vect, q)
% rotate a 3D point about a normalized quaternion
% input: array of vectors (3 x n), quaternion (4 x 1)
% output: array of rotated vectors (3 x n)

n = size(vect,2);
p = [zeros(1,n);vect(1,:);vect(2,:);vect(3,:)]; % (4 x n)

q_conj = [q(1);-q(2);-q(3);-q(4)];

rotated_point = sym(zeros(3,n));
for i=1:n
    p_tmp = quat_mult(p(:,i),q_conj);
    p_rotated = quat_mult(q, p_tmp);
    rotated_point(:,i) = p_rotated(2:4);
end
