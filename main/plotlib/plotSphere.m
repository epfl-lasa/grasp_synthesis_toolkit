function plotSphere(object, no_lights, clr_rgb)

if nargin<2
    no_lights = false;
end

if nargin<3
    if ~isfield(object, 'clr')
        clr_rgb = [0.9290 0.6940 0.1250];
    else
        clr_rgb = object.clr;
    end
end

surf(object.X,object.Y,object.Z,'FaceColor',clr_rgb,'FaceAlpha',0.75,'EdgeColor','k','EdgeAlpha',0.5,'FaceLighting','gouraud');
if ~no_lights
    light('Position',[100 0 100],'Style','local')
    light('Position',[-100 0 100],'Style','local')
end
axis('equal')
hold on;
end