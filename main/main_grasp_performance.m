% main script to perform a grasp
clc;
% adding paths to the workspace
setup_path;
% generate problem configuration
setup_problem_config;

%% Configuration of experiment
recon.hand_model = true; % reconstruct hand models [TODO] remove this after changes applied
recon.object_model = true; % reconstruct object models
recon.rmap = true; % reconstruct reachability maps
recon.os = true; % reconstruct opposition space

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
nb_objects = 5;  % number of random initializations per object shape
switch obj_type
    case 'sph'
        param_20.radius = 20;
        sph_20 = create_object_list(obj_type, param_20, nb_objects);
        
        param_30.radius = 30;
        sph_30 = create_object_list(obj_type, param_30, nb_objects);
        
        param_10.radius = 10;
        sph_10 = create_object_list(obj_type, param_10, nb_objects);
        
        % concatenate all objects into 1 list (size 5 x 5)
        object_list = cat(2, sph_10, sph_10,sph_10, sph_20, sph_20,sph_20, sph_30);
        
    case 'cyl'
        %param_R10H30.radius = 10;
        %param_R10H30.height = 30;
        %cyl_R10H30 = create_object_list(obj_type, param_R10H30, nb_objects);
        param_R15H50.radius = 15;
        param_R15H50.height = 50;
        cyl_R15H50 = create_object_list(obj_type, param_R15H50, nb_objects);
        %object_list = cat(2, cyl_R10H30,cyl_R10H30,cyl_R10H30,...
        %                     cyl_R15H50,cyl_R15H50,cyl_R15H50);
        object_list = cat(2,cyl_R15H50,cyl_R15H50);
    case 'comp'
        param1.radius = 10;
        param1.height = 30;
        param1.sphereCenter = [0,0,25;0,0,-25];
        param1.sphereRadius = [15,15];
        comp_1 = create_object_list(obj_type, param1, nb_objects);
        
        object_list = cat(2,comp_1,comp_1);
%         object = compObj(Param);
%         mySGplotHand(hand);
%         plotCompObject(object,false);
end


% object_list = create_object_list(obj_type, Param, nb_objects);
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
switch (obj_type)
    case 'sph'
        os_list = {{[1,4],[2,3]},{[3,4],[4,4]},{[3,4],[0,0]},...  % radius 10
                   {[1,4],[2,3]},{[3,4],[4,4]},{[3,4],[0,0]},...  % radius 20
                   {[1,4],[2,4]}}; % radius 32: almost no other opposition space seems to work
    case 'cyl'
        %os_list = {{[1,4],[2,3]},{[2,3],[3,3]},{[0,0],[2,4]},...  % radius 10 height 30
        %           {[1,4],[2,3]},{[2,3],[3,3]},{[0,0],[2,4]}};    % radius 15 height 50
        os_list = {{[2,3],[3,3]}};       
    case 'comp'
        os_list = {{[2,3],[3,3]}}  % object 1
                   %{[1,4],[2,3]},{[2,4],[3,4]},{[2,3],[2,4]}}     % object 2
end
% successful simulations achieved for:
% {[0,0],[2,4]} % radius: 10, height: 30
% {[0,0],[3,4]} % radius: 18, height: 30
% {[2,4],[3,4]} % radius: 14, height: 30
% {[2,3],[3,3]} % radius: 14, height: 30
% {[2,2],[3,2]} % radius: 14, height: 30
% not working:
% sphere - radius 10: {[1,3],[2,2]}
% sphere - radius 30: {[],[]}
logfile_title = ['perf_eval_', obj_type, '.csv'];
logfile = fopen(logfile_title,'w');
if strcmp(obj_type, 'sph')
    fprintf(logfile,'obj_type,os1,os2,obj_r,success,obj_val,nb_eq_const,n_ineq_const,const_time,opt_time\n');
else
    fprintf(logfile,'obj_type,os1,os2,obj_r,obj_h,success,obj_val,nb_eq_const,nb_ineq_const,const_time,opt_time\n');
end

for i = 1:numel(os_list) % loop over os_pair
    os_pair = os_list{i};       % select the os_pair
    nb_errors = 0;
    
    %% Configuration of experiment
    recon.hand_model = true; % reconstruct hand models [TODO] remove this after changes applied
    recon.object_model = true; % reconstruct object models
    recon.rmap = true; % reconstruct reachability maps
    recon.os = true; % reconstruct opposition space
    
    for j=1:nb_objects
        experiment_cnt = (i-1)*nb_objects + j;
        object = object_list{(i-1)*nb_objects + j};
        fprintf('OS [%d,%d] [%d,%d] Radius %d\n', os_pair{1}(1),os_pair{1}(2),...
            os_pair{2}(1),os_pair{2}(2),object.radius);
        fprintf('Experiment: %d\n', experiment_cnt);
        
        if experiment_cnt>1
            hand = reset_hand(); % reload the hand model, if previous runs exist
        end

        file_title = ['grasp_perf'...
            '_F',num2str(os_pair{1}(1)),'L',num2str(os_pair{1}(2)),...
            '_F',num2str(os_pair{2}(1)),'L',num2str(os_pair{2}(2)),...
            '_type_',object.type,'_r_',num2str(object.radius),'_run_',num2str(j)];
        disp(file_title);

        % Solve grasping synthesis optimization problem
        
        [hand, object, opt_soln, opt_cost, if_solution] = graspSingleObject(hand, object, recon, os_pair, false, false, file_title);

        % Visualize and save results
        if if_solution
            visualizeOptimizationConfig(hand, object, opt_soln.X_sol, opt_soln.param);
            save(['../database/results/',file_title,'.mat']);
            savefig(['../database/results/',file_title,'.fig']);
            eq_const = sum(opt_soln.param.c_idx);
            ineq_const = sum(opt_soln.param.ceq_idx);
            constraint_time = opt_soln.time_constraints;
            optimizer_time = opt_soln.time_optimizer;
            success = 1;
        else
            disp('No solution obtained. Exit.');
            nb_errors = nb_errors + 1;
            opt_cost = -1;
            eq_const = -1;
            ineq_const = -1;
            constraint_time = -1;
            optimizer_time = -1;
            success = 0;
        end
        
        if strcmp(object.type, 'sph')
            fprintf(logfile, '%s,F%dL%d,F%dL%d,%5.1f,%d,%8.2f,%d,%d,%5.0f,%5.0f\n',object.type, os_pair{1}(1),...
                    os_pair{1}(2),os_pair{2}(1),os_pair{2}(2), object.radius, success,...
                    opt_cost, eq_const, ineq_const, constraint_time, optimizer_time);
        else
            fprintf(logfile, '%s,F%dL%d,F%dL%d,%5.1f,%5.1f,%d,%8.2f,%d,%d,%5.0f,%5.0f\n',object.type, ...
                os_pair{1}(1),os_pair{1}(2),os_pair{2}(1),os_pair{2}(2),...
                object.radius, object.height, success, opt_cost, eq_const,ineq_const,constraint_time, optimizer_time);
        end
        
        %% Configuration of experiment
        recon.hand_model = false; % reconstruct hand models [TODO] remove this after changes applied
        recon.object_model = false; % reconstruct object models
        recon.rmap = false; % reconstruct reachability maps
        recon.os = false; % reconstruct opposition space
    end
end
fclose(logfile);
