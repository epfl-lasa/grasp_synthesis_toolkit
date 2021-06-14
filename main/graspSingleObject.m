function [hand, object, opt_soln, opt_cost, if_solution] = graspSingleObject(hand, object, recon, os_pair, if_plot, if_save, file_title, grasped_objects)
    
    if nargin < 8
        grasped_objects = {};
    end
    if nargin < 6
        if_save = false;
    end
    if nargin < 5
        if_plot = false;
    end
    
    load('../database/problem_config.mat');
    reset_symbolic_functions();
    
    %hand = activateLinkContact(hand);

    %% Step 1: Construct/load reachability map

    disp('[1] Constructing hand reachability map...');

    active_fingers = zeros(1,4);
    if os_pair{1}(1) && hand.factv(os_pair{1}(1))
        active_fingers(os_pair{1}(1)) = 1;
    end
    if os_pair{2}(1) && hand.factv(os_pair{2}(1))
        active_fingers(os_pair{2}(1)) = 1;
    end
    [rmap, hand] = rMapActiveLinks(hand, active_fingers);
    disp('Updated link reachability map in hand model');
    
    if if_plot
        plotRmapPerFinger(hand, rmap);
    end

% DEPRECATED

%     %% Step 2: Construct collision map
%     if recon.os || ~isfield(hand,'collision_map')
%         disp('Constructing collision map...');
%         hand = collisionOSSearch(hand, rmap, if_plot); % collision opposition space
%         %save('../database/models.mat','hand');
%         disp('Hand model updated: link reachability map');
%     end

    %% Step 3: Filter potential feasible opposition spaces for grasping
    if recon.os || ~isfile(['../database/opposition_space_',num2str(object.radius),'.mat'])
        disp('[2] Searching for potential opposition space...');
        OS = fitInOppsitionSpace(hand, object, rmap, if_plot);
        %[OS, existence_heatmap] = fitInOppsitionSpace(hand, object, rmap, if_plot);
        fprintf('\n Opposition space constructed.\n');
    else
        os_data = load(['opposition_space_',num2str(object.radius),'.mat']);
        OS = os_data.OS;
        fprintf('\n[2] Opposition space loaded.\n');
    end

    osCandList = selectSingleOS(OS,os_pair, object); % pre-assigned OS in the function as default and for test
    
    if isempty(osCandList)
        warning('Opposition space is not feasible')
    end
    
   %% Step 4: collision list
%    if recon.os || isfield(hand, 'collision_map')
%         disp('constructing reduced reachability map for collision search');
%         active_fingers = zeros(1,4);
%         if os_pair{1}(1) && hand.factv(os_pair{1}(1))
%             active_fingers(os_pair{1}(1)) = 1;
%         end
%         if os_pair{2}(1) && hand.factv(os_pair{2}(1))
%             active_fingers(os_pair{2}(1)) = 1;
%         end
%         [rmap_coll, hand] = rMapActiveLinks(hand, active_fingers);
     fprintf("\n[3] Searching for possible interlink collisions\n");
     do_plot = false;
     hand = collisionOSSearch(hand, rmap, do_plot);
     fprintf("Updated collision lists\n")
%    end

    %% Launch the optimization
    
    solnSet = {}; % to save results for planned grasp in each oppospc
    costList = []; % to save cost
    opt_soln = []; % overall optimal solution
    
    for k = 1:numel(osCandList)
        disp('**************************************************');
        fprintf('* Start Optimization Subproblem: %d / %d\n', k, numel(osCandList));
        os = osCandList{k};

        fprintf('  Opp. Space: F%dL%d and F%dL%d\n', os.os_info{1}(1),os.os_info{1}(2),os.os_info{2}(1),os.os_info{2}(2));

        [X_sol, Cost, param, if_trial_success] = optGraspObject(hand, object, os, grasped_objects); % This is the main function for optimal grasping synthesis

        if if_trial_success
            costList(end+1) = Cost;
            solnSet{end+1} = struct('X_sol',X_sol,'param',param);
        else
            fprintf('* No feasible solution for os candidate %d\n', k);
        end
        disp('* Optimization completed.');
    end

    if ~isempty(costList)
        if_solution = true;
        [opt_cost, opt_idx] = min(costList);
        opt_soln = solnSet{opt_idx};
        opt_soln.time_constraints = param.time_constraint;
        opt_soln.time_optimizer = param.time_optimizer;
        hand = updateHandConfig(hand, opt_soln);
        object = updateObjectConfig(object, opt_soln);
        
        disp('* Optimal solution for object found.');
        
        if if_plot
            visualizeOptimizationConfig(hand, object, X_sol, param, 'Optimal Solution of Single Grasp'); % Visualization of optimization results
        end
    else
        if_solution = false;
        opt_cost = 0;
        fprintf('* No feasible solution for object.\n');
    end
    
    if if_save
        if file_title
            save(['../database/results/',file_title,'.mat'])
        else
            save(['../database/results/single_grasp_',num2str(object.radius),'.mat']);
        end
        disp('* Optimization result saved.');
    end
    
    disp('[END] Optimizations are completed.');
end