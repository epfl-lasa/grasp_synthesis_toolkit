function plotCylinder(cyl)
% plot the cylinder object

hold on
grid on
axis('equal')
surf(cyl.X,cyl.Y,cyl.Z)

axis('equal')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

end

