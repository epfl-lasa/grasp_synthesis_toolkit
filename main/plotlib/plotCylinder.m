function plotCylinder(cyl,plotApprox,clr_rgb)
% plot the cylinder object
if nargin<3
    clr_rgb = [0,0,1];
end
hold on
grid on
axis('equal')
clr(:,:,1) = clr_rgb(1)*ones(size(cyl.xArray));
clr(:,:,2) = clr_rgb(2)*ones(size(cyl.xArray));
clr(:,:,3) = clr_rgb(3)*ones(size(cyl.xArray));


if plotApprox
    % plot the cylinder as transparent surface, with approximating spheres
    % inside
    surf(cyl.xArray,cyl.yArray,cyl.zArray,clr,'FaceAlpha',0.3);
    patch(cyl.xArray(1,:),cyl.yArray(1,:), cyl.zArray(1,:),'b','FaceAlpha',0.3);
    patch(cyl.xArray(2,:),cyl.yArray(2,:), cyl.zArray(2,:),'b','FaceAlpha',0.3);

    nSpheres = size(cyl.axPtArray,2);
    % plot the approximating spheres inside
    for i=1:nSpheres
        [x,y,z] = sphere;
        x = cyl.axPtArray(1,i) + x*cyl.radius;
        y = cyl.axPtArray(2,i) + y*cyl.radius;
        z = cyl.axPtArray(3,i) + z*cyl.radius;

        col(:,:,1) = 0.5*ones(size(x));
        col(:,:,2) = zeros(size(x));
        col(:,:,3) = zeros(size(x));
        surf(x,y,z,col,'FaceAlpha',0.6);
    end
else
    % plot the cylinder surface without transparency
    surf(cyl.xArray,cyl.yArray,cyl.zArray,clr);
    patch(cyl.xArray(1,:),cyl.yArray(1,:), cyl.zArray(1,:),'b');
    patch(cyl.xArray(2,:),cyl.yArray(2,:), cyl.zArray(2,:),'b');
end

axis('equal')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

end

