setup_path;

setup_problem_config;

%%
recon.hand_model = false; % reconstruct hand models
recon.rmap = true; % reconstruct reachability maps
recon.os = true; % reconstruct opposition space

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
else
    models = load('models.mat');
    hand = models.hand;
    fprintf('\n[1] Hand model loaded.\n');
end


obj_radius = 15;
object = sphereObject([0;0;-100],obj_radius);

%% Step 1: Construct/load reachability map
if recon.rmap || ~isfield(hand, 'reachability_map')
    disp('Constructing hand reachability map...');
    [rmap, hand] = reachabilityMap(hand, 'all', false);
    fprintf('\n[2] Hand reachability map constructed.\n');
    %save('../database/models.mat','hand');
    disp('Hand model updated: link reachability map');
else
    rmap = load('reachable_map.mat');
    rmap = rmap.map_hand;
    fprintf('\n[2] Hand reachability map loaded.\n');
end

plotRmapPerFinger(hand, rmap);

%% Step 2: Construct collision map

fprintf('\n[3]Constructing collision map...\n');
[hand, coll_map] = collisionOSSearch(hand, rmap, false); % collision opposition space
fprintf('Hand model updated: link collision map\n');

%% Create a list with all opposition spaces

% should be a priori in the opposition space (no need to check that later)

%% Step 3: Filter potential feasible opposition spaces for grasping
if recon.os || ~isfile(['../database/opposition_space_',num2str(object.radius),'.mat'])
    disp('Searching for potential opposition space...');
    os_list = fitInOppsitionSpace(hand, object, rmap, true);
    %[OS, existence_heatmap] = fitInOppsitionSpace(hand, object, rmap, if_plot);
    fprintf('\n[4] Opposition space constructed.\n');
else
    os_data = load(['opposition_space_',num2str(object.radius),'.mat']);
    os_list = os_data.OS;
    fprintf('\n[4] Opposition space loaded.\n');
end

if isempty(os_list)
    warning('No OS candidate found')
end
    
cnt_solutions = 0;
costList = [];
solnSet = {};
successful_os = {};

for k = 1:numel(os_list)
    
    % Reload hand and object
    if ~exist('hand')
        models = load('models.mat');
        hand = models.hand;
    end
    if ~exist('object')
        obj_radius = 15;
        object = sphereObject([0;0;-100],obj_radius);
    end
    
    disp('**************************************************');
    fprintf('* Start optimizing OS %d  of %d\n', k, numel(os_list));
    os = os_list{k};

    fprintf('  Opp. Space: F%dL%d and F%dL%d\n', os.os_info{1}(1),os.os_info{1}(2),os.os_info{2}(1),os.os_info{2}(2));

    [X_sol, Cost, param, trial_success] = optGraspObject(hand, object, os); % main function for optimal grasping synthesis

    if trial_success
        successful_os{end+1} = os.os_info;
        costList(end+1) = Cost;
        opt_soln = struct('X_sol',X_sol,'param',param);
        solnSet{end+1} = opt_soln;
        hand = updateHandConfig(hand, opt_soln);
        object = updateObjectConfig(object, opt_soln);
        fprintf('* Optimal solution for object found.\n');
        
        file_name = ['../database/results_os/single_grasp_OS_',num2str(k)];
        
        % visualize the updated Hand
        visualizeOptimizationConfig(hand, object, X_sol, param, 'Optimal Solution of Single Grasp'); % Visualization of optimization results
        
        % save the figure and variables
        fig_file_name = [file_name,'.fig'];
        saveas(gcf, fig_file_name)
        save([file_name,'.mat'])
        
        fprintf('* Optimization result saved.\n');        
    else
        fprintf('* No feasible solution for os candidate %d\n', k);
    end
    
    
    %%% DELETE ALL OLD FUNCTIONS
    % if one file exists, delete all
    if isfile('../database/symbolic_functions/nonl_c.m')
        delete ../database/symbolic_functions/nonl_c.m
        functions_exist = true;
    end
    if isfile('../database/symbolic_functions/nonl_c_grad.m')
        delete ../database/symbolic_functions/nonl_c_grad.m
        functions_exist = true;
    end

    % nonlinear equality constraints
    if isfile('../database/symbolic_functions/nonl_ceq.m')
        delete ../database/symbolic_functions/nonl_ceq.m
        functions_exist = true;
    end

    if isfile('../database/symbolic_functions/nonl_ceq_grad.m')
        delete ../database/symbolic_functions/nonl_ceq_grad.m
        functions_exist = true;
    end

    % objective function
    if isfile('../database/symbolic_functions/objfun.m')
        delete ../database/symbolic_functions/objfun.m
        functions_exist = true;
    end
    if isfile('../database/symbolic_functions/objfun_grad.m')
        delete ../database/symbolic_functions/objfun_grad.m
        functions_exist = true;
    end

    if functions_exist
        fprintf('\nDeleted objective and constraint functions\n')
    else
        fprintf('\nNo objective and constraint to delete\n')
    end
    clear hand object
    fprintf('\n* Optimization %d  of %d completed\n', k, numel(os_list));
end

fprintf('\n[END] Optimizations are completed.\n');

