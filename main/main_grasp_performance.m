% main script to perform a grasp
clc;
% adding paths to the workspace
setup_path;
% generate problem configuration
setup_problem_config;

%% Configuration of experiment
recon.hand_model = false; % reconstruct hand models [TODO] remove this after changes applied
recon.object_model = true; % reconstruct object models
recon.rmap = false; % reconstruct reachability maps
recon.os = false; % reconstruct opposition space

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
obj_type = 'comp';
switch obj_type
    case 'sph'
        R = [20,32,10];
        idx = 1;
        Param.radius = R(idx);
    case 'cyl'
        R = [14,16,20];
        H = [30,40,50];
        idx = 1;
        Param.radius = R(idx);
        Param.height = H(idx);
    case 'comp'
        Param.radius = 15;
        Param.height = 30;

        Param.sphereCenter = [0,0,25;0,0,-25];
        Param.sphereRadius = [20,20];
%         object = compObj(Param);
%         mySGplotHand(hand);
%         plotCompObject(object,false);
end
nb_objects = 2;
object_list = create_object_list(obj_type, Param, nb_objects);
fprintf("[2] Object list of type %s constructed.\n", obj_type);
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

os_list = {{[1,4],[2,3]},{[2,4],[3,4]},{[0,0],[2,4]}};%,...
% successful simulations achieved for:
% {[0,0],[2,4]} % radius: 10, height: 30
% {[0,0],[3,4]} % radius: 18, height: 30
% {[2,4],[3,4]} % radius: 14, height: 30
% {[2,3],[3,3]} % radius: 14, height: 30
% {[2,2],[3,2]} % radius: 14, height: 30
% [deprecated]
% {[2,3],[3,3]}
% {[2,4],[3,4]}
% {[0,0],[1,4]}
% {[0,0],[2,4]}
% {[1,4],[2,2]}     
% failed:
% {[1,4],[2,3]}
% {[2,2],[4,2]},...
% {[2,2],[2,4]}};
logfile_title = ['perf_eval_',obj_type,'_R_',num2str(object_list{1}.radius),'.csv'];
logfile = fopen(logfile_title,'w');
fprintf(logfile,'obj_type, os_1, os_2, obj_radius[mm], obj_height[mm], obj_value, time[s]\n');
for i = 1:numel(os_list) % loop over os_pair
    os_pair = os_list{i};       % select the os_pair
    nb_errors = 0;
    obj_fun_value = zeros([1,numel(object_list)]);
    optimizer_time = zeros([1,numel(object_list)]);
    for j=1:numel(object_list)
        experiment_cnt = (i-1)*j + j
        fprintf('Experiment: %d\n', experiment_cnt);
        if experiment_cnt>1
            hand = reset_hand(); % reload the hand model, if previous runs exist
        end

        object = object_list{j};    % select the object


        file_title = ['grasp_perf'...
            '_F',num2str(os_pair{1}(1)),'L',num2str(os_pair{1}(2)),...
            '_F',num2str(os_pair{2}(1)),'L',num2str(os_pair{2}(2)),...
            '_type_',object.type,'_r_',num2str(object.radius),'_run_',num2str(j)];
        disp(file_title);

        % Solve grasping synthesis optimization problem
        
        [hand, object, opt_soln, opt_cost, if_solution] = graspSingleCylinder(hand, object, recon, os_pair, false, false, file_title);

        % Visualize and save results
        if if_solution
            visualizeOptimizationConfig(hand, object, opt_soln.X_sol, opt_soln.param);
            save(['../database/results/',file_title,'.mat']);
            savefig(['../database/results/',file_title,'.fig']);
            obj_fun_value(j) = opt_cost;
            constraint_time = opt_soln.time_constraints;
            optimizer_time = opt_soln.time_optimizer;
            success = 1;
        else
            disp('No solution obtained. Exit.');
            nb_errors = nb_errors + 1;
            opt_cost = -1;
            constraint_time = -1;
            optimizer_time = -1;
            success = 0;
        end
        
        if strcmp(object.type, 'sph')
            fprintf(logfile, '%s,F%dL%d, F%dL%d, %5.1f, %d, %8.2f, %5.0f, %5.0f\n',object.type, os_pair{1}(1),...
                    os_pair{1}(2),os_pair{2}(1),os_pair{2}(2), object.radius, success,...
                    opt_cost, constraint_time, optimizer_time);
        else
            fprintf(logfile, '%s, F%dL%d, F%dL%d, %5.1f, %5.1f, %d, %8.2f, %5.0f, %5.0f\n',object.type, ...
                os_pair{1}(1),os_pair{1}(2),os_pair{2}(1),os_pair{2}(2),...
                object.radius, object.height, success, opt_cost, constraint_time, optimizer_time);
        end
    end
end
fclose(logfile);
