function [object_list] = create_object_list(obj_type, param, nb_objects, del)
% create a list of objects
% NOTE: the shape of first 3 object is identical

if nargin<4
    % parameters of the center position
    del.x_min = -60;
    del.x_max = 60;
    del.y_min = -80;
    del.y_max = 40;
    del.z_min = -100;
    del.z_max = 0; 
end

object_list = cell(1,nb_objects);
switch obj_type
    case 'sph'
        radius = param.radius;
        t_x = (del.x_max-del.x_min)*rand([nb_objects,1])+del.x_min;
        t_y = (del.y_max-del.y_min)*rand([nb_objects,1])+del.y_min;
        t_z = (del.z_max-del.z_min)*rand([nb_objects,1])+del.z_min;
        for i=1:nb_objects
            object = sphereObject([t_x(i);t_y(i);t_z(i)], radius);
            object_list{i} = object;
        end
    case 'cyl'
        p.radius = param.radius;
        p.height = param.height;
        roll = 180 * rand([nb_objects,1]) - 90; % roll in [-90,90]°
        pitch = 180 * rand([nb_objects,1]) - 90; % pitch in [-90,90]°
        yaw = zeros(nb_objects,1); % yaw is irrelevant due to symmetry
        t_x = (del.x_max-del.x_min)*rand([nb_objects,1])+del.x_min;
        t_y = (del.y_max-del.y_min)*rand([nb_objects,1])+del.y_min;
        t_z = (del.z_max-del.z_min)*rand([nb_objects,1])+del.z_min;
        for i=1:nb_objects
           p.quat = quaternion([yaw(i), pitch(i), roll(i)],'euler','ZYX','frame');
           p.transl = [t_x(i);t_y(i);t_z(i)];
           p.roll = roll(i); p.pitch = pitch(i); p.yaw = 0;
           object = cylinderObject(p);
           object_list{i} = object;
        end
    case 'comp'
        p.radius = param.radius;
        p.height = param.height;
        roll = 180 * rand([nb_objects,1]) - 90; % roll in [-90,90]°
        pitch = 180 * rand([nb_objects,1]) - 90; % pitch in [-90,90]°
        yaw = zeros(nb_objects,1); % yaw is irrelevant due to symmetry
        t_x = (del.x_max-del.x_min)*rand([nb_objects,1])+del.x_min;
        t_y = (del.y_max-del.y_min)*rand([nb_objects,1])+del.y_min;
        t_z = (del.z_max-del.z_min)*rand([nb_objects,1])+del.z_min;
        for i=1:nb_objects
           p.quat = quaternion([yaw(i), pitch(i), roll(i)],'euler','ZYX','frame');
           p.transl = [t_x(i);t_y(i);t_z(i)];
           p.roll = roll(i); p.pitch = pitch(i); p.yaw = 0;
           p.sphereCenter = param.sphereCenter;
           p.sphereRadius = param.sphereRadius;
           object = compObject(p);
           object_list{i} = object;
        end
        
end
    
end

