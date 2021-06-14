function plotCompObject(object,plotApprox,no_lights, clr_rgb)


if nargin<4 && ~isfield(object, 'clr')
    clr_rgb = [0.9290 0.6940 0.1250];
else
    clr_rgb = object.clr;
end
if nargin < 3
    no_lights = true;
end
% plot the cylinder object

if nargin<2
    plotApprox = false;
end
hold on
grid on
axis('equal')
clr(:,:,1) = clr_rgb(1)*ones(size(object.xArray));
clr(:,:,2) = clr_rgb(2)*ones(size(object.xArray));
clr(:,:,3) = clr_rgb(3)*ones(size(object.xArray));

if ~no_lights
    light('Position',[100 0 100],'Style','local')
    light('Position',[-100 0 100],'Style','local')
    light('Position',[50 200 100],'Style','local')
    light('Position',[0 0 -100],'Style','local')
end
%material dull 

if plotApprox
    % plot the cylinder as transparent surface, with approximating spheres
    % inside
    surf(object.xArray,object.yArray,object.zArray,clr,'FaceAlpha',0.3);
    patch(object.xArray(1,:),object.yArray(1,:), object.zArray(1,:),clr_rgb,'FaceAlpha',0.3);
    patch(object.xArray(2,:),object.yArray(2,:), object.zArray(2,:),clr_rgb,'FaceAlpha',0.3);

    nSpheres = size(object.axPtArray,2);
    % plot the approximating spheres inside
    for i=1:nSpheres
        [x,y,z] = sphere;
        x = object.axPtArray(1,i) + x*object.radius;
        y = object.axPtArray(2,i) + y*object.radius;
        z = object.axPtArray(3,i) + z*object.radius;

        col(:,:,1) = 1*ones(size(x));
        col(:,:,2) = zeros(size(x));
        col(:,:,3) = zeros(size(x));
        surf(x,y,z,col,'FaceAlpha',0.6);
    end
    
    for i=1:length(object.sphereRadius)
        [x,y,z] = sphere;
        x = x*object.sphereRadius(i) + object.sphereCenter(1,i);
        y = y*object.sphereRadius(i) + object.sphereCenter(2,i);
        z = z*object.sphereRadius(i) + object.sphereCenter(3,i);
        clr = [];
        clr(:,:,1) = clr_rgb(1) * ones(size(x));
        clr(:,:,2) = clr_rgb(2) * ones(size(x));
        clr(:,:,3) = clr_rgb(3) * ones(size(x));
        
        surf(x,y,z,clr,'FaceAlpha',0.3);
    end
    
else
    if ~no_lights
        lightangle(-45,30)
    end    
    % plot the cylinder surface without transparency
    surf(object.xArray,object.yArray,object.zArray,clr,'FaceLighting','gouraud');
    patch(object.xArray(1,:),object.yArray(1,:), object.zArray(1,:),clr_rgb,'FaceLighting','gouraud');
    patch(object.xArray(2,:),object.yArray(2,:), object.zArray(2,:),clr_rgb,'FaceLighting','gouraud');
    
    for i=1:length(object.sphereRadius)
        [x,y,z] = sphere;
        x = x*object.sphereRadius(i) + object.sphereCenter(1,i);
        y = y*object.sphereRadius(i) + object.sphereCenter(2,i);
        z = z*object.sphereRadius(i) + object.sphereCenter(3,i);
        clr = [];
        clr(:,:,1) = clr_rgb(1) * ones(size(x));
        clr(:,:,2) = clr_rgb(2) * ones(size(x));
        clr(:,:,3) = clr_rgb(3) * ones(size(x));
        
        surf(x,y,z,clr,'FaceLighting','gouraud');
    end
    
    
end

axis('equal')
ax = gca;
ax.FontSize = 20; 
xlabel('x-axis','Fontsize',20)
ylabel('y-axis','Fontsize',20)
zlabel('z-axis','Fontsize',20)

end

