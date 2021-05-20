% main script to perform a grasp
clc;
% adding paths to the workspace
setup_path;
% generate problem configuration
setup_problem_config;

%% Configuration of experiment
recon.hand_model = false; % reconstruct hand models
recon.rmap = true; % reconstruct reachability maps
recon.os = true; % reconstruct opposition space

%% Create Hand Models

% if no hand model is stored, generate a the file 'model.mat'
if recon.hand_model || ~isfile('../database/models.mat')
    disp('Generating hand model...');
    Th = eye(4);
    Th(1:3,4) = [0;0;0];
    Th(1:3,1:3) = eye(3);
    
    pregrasp_rate = 0.7;
    % hand = mySGparadigmatic(Th); % hand.F{i}.idx_real_link: [0 1 1 1 0]
    hand = mySGallegroLeft(Th, pregrasp_rate);
    
    save('../database/models.mat', 'hand');
    fprintf('\n* Hand model constructed and saved.\n');
end

%% Create Object Models

%     case 'comp'
%         Param.radius = 10;
%         Param.height = 30;
%         Param.roll = 0;
%         Param.pitch = -pi/2;
%         Param.yaw = 0;
%         % bad solution: roll = pi/8, pitch = pi/6, yaw = 0, os={[2,4],[3,4]}
%         Param.quat = quaternion([Param.yaw,Param.pitch,Param.roll],'euler', 'ZYX','frame');
%         Param.transl = [0;0;-100]; % translation
%         
%         Param.sphereCenter = [0,0,25;0,0,-25];
%         Param.sphereRadius = [15,15];
%         object = compObject(Param);
%         mySGplotHand(hand);
%         plotCompObject(object,false);
% end

% load objects from 'objects_experience.mat'
% (all the required objects are in this file')

load objects_experience.mat
object_list = {};
osList = {};
% SPHERE OBJECTS

%%% small spheres
% object_list = [object_list, small_sph, small_sph, small_sph];
% osList = [osList, {{[2,4],[3,4]}}, {{[1,4],[2,3]}}, {{[2,4],[0,0]}}];

%%% medium spheres
% object_list = [object_list, med_sph, med_sph, med_sph];
% osList  = [osList, {{[2,4],[4,4]}}, {{[1,4],[2,3]}}, {{[2,4],[0,0]}}];

%%% big spheres
% object_list = [object_list, big_sph, big_sph, big_sph];
% osList  = [osList, {{[2,4],[4,4]}}, {{[1,4],[2,3]}}, {{[2,4],[0,0]}}];


% CYLINDER OBJECTS
%%% small cylinder

% object_list = [object_list, small_cyl, small_cyl, small_cyl];
% osList  = [osList, {{[2,4],[4,4]}}, {{[1,4],[2,3]}}, {{[2,4],[0,0]}}];

object_list = [object_list, long_cyl, long_cyl, long_cyl];
osList  = [osList, {{[2,4],[3,4]}}, {{[1,4],[2,3]}}, {{[2,4],[0,0]}}];

object_list = [object_list, med_cyl, med_cyl, med_cyl];
osList  = [osList, {{[2,4],[4,4]}}, {{[1,4],[2,3]}}, {{[2,4],[0,0]}}];

object_list = [object_list, big_cyl, big_cyl, big_cyl];
osList  = [osList, {{[2,4],[4,4]}}, {{[1,4],[2,3]}}, {{[2,4],[0,0]}}];

% COMPOSITE OBJECTS
% object_list = [object_list, comp1, comp1];
% osList = [osList, {{[2,4],[3,4]}},{{[1,4],[2,3]}}];

% object_list = [object_list, comp2, comp2, comp2];
% osList = [osList, {{[2,4],[3,4]}},{{[1,4],[2,3]}},{{[2,4],[0,0]}}];

%% Optimization

% List of Opposition Space pairs, used as candidates for grasping.
% The so-called "Opposition Space" is the region between two (or more)
% finger patches. In the following list, each element of the big cell
% list `osList` is also a cell element that contains two vectors with
% entries in each vector indicate the finger link name. For example  the
% first pair of OS is {[1,4],[2,4]}, meaning that this OS is the space
% region spanned by two finger patches: (1) [1,4] - the 1st finger
% (thumb), 4th link (dobject)istal phalanx), and (2) [2,4] - the 2nd finger
% (index finger), 4th link (distal phalanx).
% Index 1-5 correspond to thumb, index, middle, ring, and little fingers,
% for a human paradigmatic hand model.
% For any other hand models, finger indexing starts from thumb.
% Within each finger, links are indexed from the bottom of the finger to 
% the tip of the finger.
% However, link '1' is usually modeled as a virtual link (length 0) that 
% comprises the ad-/abduction degrees of freedom on the bottom of the finger.
% The last link is used to model another virtual link at finger tip for convenience.

% logfile generation


for i = 1:numel(osList)
    
    fprintf('\n***************************\nExperiment: %d\n***************************\n', i);
    os_pair = osList{i};
    object = object_list{i};
    

    
    % reload the hand
    
    % this is necessary after each optimizations, since the hand will be
    % updated according to the previous solution (for plotting).
    models = load('models.mat');
    hand = models.hand;
    fprintf('\n[1] Hand model loaded.\n');   
    
    file_name = ['experience_object_',object.type, '_R', num2str(object.radius), 'F', num2str(os_pair{1}(1)), 'L', num2str(os_pair{1}(1)), ...
                 'F', num2str(os_pair{2}(1)), 'L', num2str(os_pair{2}(1))];


    % Solve grasping synthesis optimization problem
    [hand, object, opt_soln, opt_cost, if_solution] = graspSingleObject(hand, object, recon, os_pair, false, false, file_name);

    % Visualize and save results
    if if_solution
        visualizeOptimizationConfig(hand, object, opt_soln.X_sol, opt_soln.param);
        
        save(['../database/results/',file_name,'.mat']);
        savefig(['../database/results/',file_name,'.fig']);
        
            % generate a logfile
%         file_name = [object.type, '_R', num2str(object.radius), 'F', num2str(os_pair{1}(1)), 'L', num2str(os_pair{1}(1)), ...
%                  'F', num2str(os_pair{2}(1)), 'L', num2str(os_pair{2}(1)),'txt'];
        logfile_name = [file_name, '.txt'];
        logfile = fopen(logfile_name,'w');
        fprintf(logfile, '%f,', hand.q(:));
        fprintf(logfile, '-,%f, %f, %f\n', object.ctr(:));
        fclose(logfile);
    else
        disp('No solution obtained. Exit.');
    end
end