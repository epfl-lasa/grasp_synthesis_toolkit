function plotCylinder(cyl)
% plot the cylinder object

figure
title('Cylinder object')
surf(cyl.X,cyl.Y,cyl.Z)

axis('equal')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

end

