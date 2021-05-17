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
    if recon.rmap || ~isfield(hand, 'reachability_map')
        disp('Constructing hand reachability map...');
        [rmap, hand] = reachabilityMap(hand, 'all', false);
        fprintf('\n[3] Hand reachability map constructed.\n');
        %save('../database/models.mat','hand');
        disp('Hand model updated: link reachability map');
    else
        rmap = load('reachable_map.mat');
        rmap = rmap.map_hand;
        fprintf('\n[3] Hand reachability map loaded.\n');
    end
    if true
        plotReachabilityMap(hand, rmap);
    end
    
    %% Step 2: Construct collision map
    if recon.os || ~isfield(hand,'collision_map')
        disp('Constructing collision map...');
        hand = collisionOSSearch(hand, rmap, if_plot); % collision opposition space
        %save('../database/models.mat','hand');
        disp('Hand model updated: link reachability map');
    end

    %% Step 3: Filter potential feasible opposition spaces for grasping
    if recon.os || ~isfile(['../database/opposition_space_',num2str(object.radius),'.mat'])
        disp('Searching for potential opposition space...');
        OS = fitInOppsitionSpace(hand, object, rmap, if_plot);
        %[OS, existence_heatmap] = fitInOppsitionSpace(hand, object, rmap, if_plot);
        fprintf('\n[4] Opposition space constructed.\n');
    else
        os_data = load(['opposition_space_',num2str(object.radius),'.mat']);
        OS = os_data.OS;
        fprintf('\n[4] Opposition space loaded.\n');
    end

    osCandList = selectSingleOS(OS,os_pair); % pre-assigned OS in the function as default and for test
    
    if isempty(osCandList)
        warning('No OS candidate found')
    end
    
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