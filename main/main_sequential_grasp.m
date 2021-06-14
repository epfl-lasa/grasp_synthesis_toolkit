% main script to perform a grasp
clc;
% adding paths to the workspace
setup_path;
% generate problem configuration
setup_problem_config;

%% Configuration of experiment
recon.hand_model = true; % reconstruct hand models
recon.rmap = true; % reconstruct reachability maps (keep true to resample after each grasp)
recon.os = true;    % reconstruct opposition space (keep true to resample after each grasp)

%% Create Hand Models
if recon.hand_model && ~exist('hand','var')
    disp('Generating hand model...');
    Th = eye(4);
    Th(1:3,4) = [0;0;0];
    Th(1:3,1:3) = eye(3);
    
    % hand = mySGparadigmatic(Th); % hand.F{i}.idx_real_link: [0 1 1 1 0]
    hand = mySGallegroLeft(Th);
    
    save('../database/models.mat', 'hand');
    fprintf('\n[1] Hand model constructed and saved.\n');
else
    models = load('models.mat');
    hand = models.hand;
    fprintf('\n[1] Hand model loaded.\n');
end
%% Create Object Models

generate_objects = true;
if generate_objects
    nb_obj_per_set = 2;
    objects = generate_object_set(nb_obj_per_set);
else
    objects = load('objects.mat')
end
% define an object manually

% type = 'sph';
% switch type
%     case 'sph'
%         Param.radius = 20;
%         transl = [0;0;0];
%         object = sphereObject(transl, Param.radius);
%     case 'cyl'
%         Param.radius = 14;
%         Param.height = 30;
%         Param.roll = 0; Param.pitch = pi/2; Param.yaw = 0;
%         Param.quat = quaternion([Param.yaw,Param.pitch,Param.roll],'euler', 'ZYX','frame');
%         Param.transl = [0;0;-60]; % translation
%         object = cylinderObject(Param);
% 
%         plotCylinder(object,false);
%     case 'comp'
%         Param.radius = 15;
%         Param.height = 30;
%         Param.roll = 0; Param.pitch = -pi/2; Param.yaw = 0;
%         Param.quat = quaternion([Param.yaw,Param.pitch,Param.roll],'euler', 'ZYX','frame');
%         Param.transl = [0;0;-100]; % translation
%         
%         Param.sphereCenter = [0,0,25;0,0,-25];
%         Param.sphereRadius = [20,20];
%         object = compObject(Param);
%         plotCompObject(object,false);
% end

% squash ball: sphere with radius 20
% tennis ball: sphere with radius 30 (works only with F1L4 and F2L4

object_list = {objects.cyl_R15H40{1}, objects.comp{1}, objects.cyl_R15H40{1}, objects.cyl_R15H40{1}};
% 1: {objects.sph15{1}, objects.cyl_R15H30{1}}
% 3: {objects.cyl_R15H30{1}, objects.sph20{1}, objects.cyl_R15H40{1}}
%% Optimization

% List of Opposition Space pairs, used as candidates for grasping.
% The so-called "Opposition Space" is the region between two (or more)
% finger patches. In the following list, each element of the big cell
% list `osList` is also a cell element that contains two vectors with
% entries in each vector indicate the finger link name. For example  the
% first pair of OS is {[1,4],[2,4]}, meaning that this OS is the space
% region spanned by two finger patches: (1) [1,4] - the 1st finger
% (thumb), 4th link (distal phalanx), and (2) [2,4] - the 2nd finger
% (index finger), 4th link (distal phalanx).
% Index 1-5 correspond to thumb, index, middle, ring, and little fingers,
% for a human paradigmatic hand model.
% For any other hand models, finger indexing starts from thumb.
% Within each finger, links are indexed from the bottom of the finger to 
% the tip of the finger.
% However, link '1' is usually modeled as a virtual link (length 0) that 
% comprises the ad-/abduction degrees of freedom on the bottom of the finger.
% The last link is used to model another virtual link at finger tip for convenience.

osList = {{[4,4],[0,0]}, {[4,3],[3,3]}, {[2,4],[3,4]}, {[1,4],[2,2]}};%,...
% 1: {{[1,4],[2,3]},{[3,3],[4,3]}}
% 3: {{[3,3],[4,3]},    {[2,3],[3,3]}, {[1,4],[2,2]}};
grasped_objects = {};
for i = 1:numel(osList)
    fprintf('Grasping object: %d\n', i);
    
    os_pair = osList{i}; % TODO pass a list of opposition spaces
    object = object_list{i};
    file_title = ['sequential_grasp_'...
        '_obj_',num2str(i),...
        '_F',num2str(os_pair{1}(1)),'L',num2str(os_pair{1}(2)),...
        '_F',num2str(os_pair{2}(1)),'L',num2str(os_pair{2}(2)),...
        '_type_',object.type];
    disp(file_title);

    % Solve grasping synthesis optimization problem
    [hand, object, opt_soln, opt_cost, if_solution] = graspSingleObject(hand, object, recon, os_pair, false, false, file_title, grasped_objects);

    % Visualize and save results
    if if_solution
        grasped_objects{end+1} = object;
        visualizeOptimizationConfig(hand, grasped_objects, opt_soln.X_sol, opt_soln.param);
        save(['../database/results_sequential/',file_title,'.mat']);
        savefig(['../database/results_sequential/',file_title,'.fig']);
    else
        disp('No solution obtained. Exit.');
        break; % stop execution if one of the objects cannot be placed
    end
end