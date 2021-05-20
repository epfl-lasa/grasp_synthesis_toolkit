function generate_experience_objects()

%%% SPHERES

% create a sphere of size of a tennis ball
big_sph = sphereObject([0,0,-100], 32);

% create a sphere with the same size of the bigger wooden sphere
med_sph = sphereObject([0;0;-100], 22.5);

% create a sphere with size of the smaller wooden sphere
small_sph = sphereObject([0;0;-100], 17.5);

%%% CYLINDERS

% cork object
par_small_cyl.radius = 12.5;
par_small_cyl.height = 45;
par_small_cyl.transl = [0;0;-100];
par_small_cyl.pitch = pi/2; par_small_cyl.roll = pi/4; par_small_cyl.yaw = 0;
par_small_cyl.quat = quaternion([par_small_cyl.yaw, par_small_cyl.pitch, par_small_cyl.roll], 'euler', 'ZYX', 'frame');
small_cyl = cylinderObject(par_small_cyl);

% medium cylinder (plastic cylinder)
par_med_cyl.radius = 24;
par_med_cyl.height = 85;
par_med_cyl.transl = [0;0;-100];
par_med_cyl.pitch = pi/2; par_med_cyl.roll = pi/4; par_med_cyl.yaw = 0;
par_med_cyl.quat = quaternion([par_med_cyl.yaw, par_med_cyl.pitch, par_med_cyl.roll],'euler', 'ZYX', 'frame');
med_cyl = cylinderObject(par_med_cyl);

% big cylinder object (cardboard cylinder)
par_big_cyl.radius = 40;
par_big_cyl.height = 55;
par_big_cyl.transl = [0;0;-100];
par_big_cyl.pitch = pi/2; par_big_cyl.roll = pi/4; par_big_cyl.yaw = 0;
par_big_cyl.quat = quaternion([par_big_cyl.yaw, par_big_cyl.pitch, par_big_cyl.roll],'euler', 'ZYX', 'frame');
big_cyl = cylinderObject(par_big_cyl);

% long cylinder object
par_long_cyl.radius = 24;
par_long_cyl.height = 85*3;
par_long_cyl.transl = [0;0;-200];
par_long_cyl.pitch = pi/2; par_long_cyl.roll = pi/4; par_long_cyl.yaw = 0;
par_long_cyl.quat = quaternion([par_long_cyl.yaw, par_long_cyl.pitch, par_long_cyl.roll],'euler', 'ZYX', 'frame');
long_cyl = cylinderObject(par_long_cyl);

%%% COMPOSITE OBJECTS

% composite object 1

par_comp1.radius = 16;
par_comp1.height = 46;
par_comp1.transl = [0;0;-200];
par_comp1.pitch = pi/2; par_comp1.roll = pi/4; par_comp1.yaw = 0;
par_comp1.quat = quaternion([par_comp1.yaw, par_comp1.pitch, par_comp1.roll],'euler', 'ZYX', 'frame');
par_comp1.sphereCenter = [0,0,40;0,0,-40];
par_comp1.sphereRadius = [22.5,22.5];
comp1 = compObject(par_comp1);

% composite object 2
par_comp2.radius = 16;
par_comp2.height = 46;
par_comp2.transl = [0;0;-200];
par_comp2.pitch = pi/2; par_comp2.roll = pi/4; par_comp2.yaw = 0;
par_comp2.quat = quaternion([par_comp2.yaw, par_comp2.pitch, par_comp2.roll],'euler', 'ZYX', 'frame');
par_comp2.sphereCenter = [0,0,35;20,0,35;-20,0,35];
par_comp2.sphereRadius = [16,16,16];
comp2 = compObject(par_comp2);

save('objects_experience.mat', 'small_sph', 'med_sph', 'big_sph',...
        'small_cyl', 'med_cyl', 'big_cyl','long_cyl',...
        'comp1','comp2');
    
fprintf('* Saved objects for experience to "objects_experience.mat"\n');
end