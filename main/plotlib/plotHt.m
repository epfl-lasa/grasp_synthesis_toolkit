function plotHt(H,clr)
% plot the reference frame N in coordinates of frame N-1
% (adapted from Kunpeng Yao)

% input:    H: holonomic transform (transforms coordinates from 
%              frame N to frame N-1
%           clr: color

if nargin < 2
    clr = ['r','g','b'];
end

orig = H(1:3,4); % origin of the reference frame N w.r.t. frame N-1
x = H(1:3,1:3)*[1;0;0]; % x axis
y = H(1:3,1:3)*[0;1;0]; % y axis
z = H(1:3,1:3)*[0;0;1]; % z axis

figure
quiver3(0,0,0,orig(1),orig(2),orig(3),['k']) % translation of the origin
hold on
% plot the frame axes
quiver3(orig(1),orig(2),orig(3),x(1),x(2),x(3),[clr(1)])
quiver3(orig(1),orig(2),orig(3),y(1),y(2),y(3),[clr(2)])
quiver3(orig(1),orig(2),orig(3),z(1),z(2),z(3),[clr(3)])
axis('equal')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
hold off

end