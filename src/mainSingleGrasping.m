function [hand, object, opt_soln, opt_cost, if_solution] = mainSingleGrasping(hand, object, recon, os_pair, if_plot, if_save, file_title)

    if nargin < 5
        if_save = false;
    end
    if nargin < 4
        if_plot = false;
    end
    
    load('../database/problem_config.mat');
    
    hand = activateLinkContact(hand);

    %% Step 1: Construct/load reachability map
    if recon.rmap || ~isfield(hand, 'reachability_map')
        disp('Constructing hand reachability map...');
        [rmap, hand] = reachabilityMap(hand, 'all', false);
        fprintf('\n[2] Hand reachability map constructed.\n');
        save('../database/models.mat', 'hand');
        disp('Hand model updated: link reachability map');
    else
        rmap = load('reachable_map.mat');
        rmap = rmap.map_hand;
        fprintf('\n[2] Hand reachability map loaded.\n');
    end
    plotReachabilityMap(hand, rmap);
    
    %% Step 2: Construct collision map
    if ~isfield(hand,'collision_map')
        disp('Constructing collision map...');
        hand = collisionOSSearch(hand); % collision opposition space
        save('../database/models.mat', 'hand');
        disp('Hand model updated: link collision map');
    end

    %% Step 3: Filter potential feasible opposition spaces for grasping
    if recon.os || ~isfile(['../database/opposition_space_',num2str(object.radius),'.mat'])
        disp('Searching for potential opposition space...');
        OS = fitInOppsitionSpace(hand, object, rmap, false);
        fprintf('\n[4] Opposition space constructed.\n');
    else
        os_data = load(['opposition_space_',num2str(object.radius),'.mat']);
        OS = os_data.OS;
        fprintf('\n[4] Opposition space loaded.\n');
    end

    osCandList = selectSingleOS(OS,os_pair); % pre-assigned OS in the function as default and for test

    solnSet = {}; % to save results for planned grasp in each oppospc
    costList = []; % to save cost
    opt_soln = []; % overall optimal solution
    
    for k = 1:numel(osCandList)
        disp('**************************************************');
        fprintf('* Start Optimization Subproblem: %d / %d\n', k, numel(osCandList));
        os = osCandList{k};

        fprintf('  Opp. Space: F%dL%d and F%dL%d\n', transpose(os.os_info));

        [X_sol, Cost, param, if_trial_success] = optGraspJS(hand, object, os); % This is the main function for optimal grasping synthesis

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