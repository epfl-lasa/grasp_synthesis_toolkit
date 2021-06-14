function [objects] =  generate_object_set(nb_obj_per_set)

fprintf("Creating sets of objects for sequential grasp...\n");

% generate spheres
par_sph_10.radius = 10;
par_sph_15.radius = 15;
par_sph_20.radius = 20;
par_sph_25.radius = 25;
par_sph_30.radius = 30;

sph_10 = create_object_list('sph',par_sph_10, nb_obj_per_set);
sph_15 = create_object_list('sph',par_sph_15, nb_obj_per_set);
sph_20 = create_object_list('sph',par_sph_20, nb_obj_per_set);
sph_25 = create_object_list('sph',par_sph_25, nb_obj_per_set);
sph_30 = create_object_list('sph',par_sph_30, nb_obj_per_set);



% generate cylinders
par_cyl_R10H30.radius = 10; par_cyl_R10H30.height = 30;
par_cyl_R15H30.radius = 15; par_cyl_R15H30.height = 30;
par_cyl_R10H40.radius = 10; par_cyl_R10H40.height = 40;
par_cyl_R15H40.radius = 15; par_cyl_R15H40.height = 40;
par_cyl_R15H50.radius = 15; par_cyl_R15H50.height = 50;

cyl_R10H30 = create_object_list('cyl',par_cyl_R10H30, nb_obj_per_set);
cyl_R15H30 = create_object_list('cyl',par_cyl_R15H30, nb_obj_per_set);
cyl_R10H40 = create_object_list('cyl',par_cyl_R10H40, nb_obj_per_set);
cyl_R15H40 = create_object_list('cyl',par_cyl_R15H40, nb_obj_per_set);
cyl_R15H50 = create_object_list('cyl',par_cyl_R15H50, nb_obj_per_set);

% generate composite objects
par_comp_1.radius = 10; par_comp_1.height = 30;
par_comp_1.sphereCenter = [0,0,25;0,0,-25];
par_comp_1.sphereRadius = [15,15];

comp1 = create_object_list('comp',par_comp_1, nb_obj_per_set);

objects.sph10 = sph_10;
objects.sph15 = sph_15;
objects.sph20 = sph_20;
objects.sph25 = sph_25;
objects.sph30 = sph_30;

objects.cyl_R10H30 = cyl_R10H30;
objects.cyl_R15H30 = cyl_R15H30;
objects.cyl_R10H40 = cyl_R10H40;
objects.cyl_R15H40 = cyl_R15H40;
objects.cyl_R15H50 = cyl_R15H50;

objects.comp = comp1;

save('objects.mat','sph_10','sph_15','sph_20','sph_25','sph_30',...
                    'cyl_R10H30','cyl_R15H30','cyl_R10H40','cyl_R15H40','cyl_R15H50',...
                    'comp1')
end