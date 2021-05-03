function plotCompObj(object,plotApprox,clr_rgb)
% plot the cylinder object
if nargin<3
    clr_rgb = [0,0,1];
end
hold on
grid on
axis('equal')
clr(:,:,1) = clr_rgb(1)*ones(size(object.xArray));
clr(:,:,2) = clr_rgb(2)*ones(size(object.xArray));
clr(:,:,3) = clr_rgb(3)*ones(size(object.xArray));


if plotApprox
    % plot the cylinder as transparent surface, with approximating spheres
    % inside
    surf(object.xArray,object.yArray,object.zArray,clr,'FaceAlpha',0.3);
    patch(object.xArray(1,:),object.yArray(1,:), object.zArray(1,:),'b','FaceAlpha',0.3);
    patch(object.xArray(2,:),object.yArray(2,:), object.zArray(2,:),'b','FaceAlpha',0.3);

    nSpheres = size(object.axPtArray,2);
    % plot the approximating spheres inside
    for i=1:nSpheres
        [x,y,z] = sphere;
        x = object.axPtArray(1,i) + x*object.radius;
        y = object.axPtArray(2,i) + y*object.radius;
        z = object.axPtArray(3,i) + z*object.radius;

        col(:,:,1) = 0.5*ones(size(x));
        col(:,:,2) = zeros(size(x));
        col(:,:,3) = zeros(size(x));
        surf(x,y,z,col,'FaceAlpha',0.6);
    end
else
    % plot the cylinder surface without transparency
    surf(object.xArray,object.yArray,object.zArray,clr);
    patch(object.xArray(1,:),object.yArray(1,:), object.zArray(1,:),'b');
    patch(object.xArray(2,:),object.yArray(2,:), object.zArray(2,:),'b');
    
    for i=1:length(object.sphereRadius)
        [X,Y,Z] = sphere;
        X = X*object.sphereRadius(i) + object.sphereCenter(1,i);
        Y = Y*object.sphereRadius(i) + object.sphereCenter(2,i);
        Z = Z*object.sphereRadius(i) + object.sphereCenter(3,i);
        clrSphere(:,:,1) = clr_rgb(1)*ones(size(X));
        clrSphere(:,:,2) = clr_rgb(2)*ones(size(X));
        clrSphere(:,:,3) = clr_rgb(3)*ones(size(X));
        surf(X,Y,Z,clrSphere);
    end

axis('equal')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

end

