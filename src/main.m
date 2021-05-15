%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Interface for planning single grasping %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
fprintf('***********************************\n');
fprintf('* Experiment: Single Object Grasp *\n');
fprintf('***********************************\n');

init;
problem_configuration;

%% Configuration of experiment
recon.hand_model = false; % reconstruct hand models
recon.object_model = false; % reconstruct object models
recon.rmap = false; % reconstruct reachability maps
recon.os = false; % reconstruct opposition space

%% Create Hand Models
if recon.hand_model
    disp('Generating hand model...');
    Th = eye(4);
    Th(1:3,4) = [0;0;0];
    Th(1:3,1:3) = eye(3);

    % hand = mySGparadigmatic(Th); % hand.F{i}.idx_real_link: [0 1 1 1 0]
    hand = mySGallegroLeft(Th);

    save('../database/models.mat', 'hand');
    fprintf('\n[1] Hand model constructed and saved: %s.\n', hand.type);
else
    models = load('models.mat');
    hand = models.hand;
    fprintf('\n[1] Hand model loaded: %s.\n', hand.type);
end

%% Create Object Models
if recon.object_model || ~exist('TargetObjects','var')
    disp('Generating objects list...');
    num_objects = 10; % total number of objects to create
    TargetObjects = cell(1,num_objects);
    for i = 1:num_objects
        TargetObjects{i} = mySGsphere(eye(4), randi([10,20])); % Construct sphere object
    end
    save('../database/objects.mat', 'TargetObjects');
else
    models = load('objects.mat');
    TargetObjects = models.TargetObjects;
    fprintf('\n[1] TargetObjects loaded.\n');
end

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

osList = {{[1,4],[2,4]}}%,...
%     {[2,4],[3,4]},...
%     {[2,3],[3,3]},...
%     {[2,2],[4,2]},...
%     {[3,4],[0,0]},...
%     {[2,2],[2,4]}};

for i = 1:numel(osList)
    fprintf('Experiment: %d\n', i);

    os_pair = osList{i};
    % object = TargetObjects{randi([1,numel(TargetObjects)])}; % pick up a random object from object lists
    object = mySGsphere(eye(4), 20);

    file_title = ['single_grasp_'...
        '_F',num2str(os_pair{1}(1)),'L',num2str(os_pair{1}(2)),...
        '_F',num2str(os_pair{2}(1)),'L',num2str(os_pair{2}(2)),...
        '_type_',object.type,'_r_',num2str(object.radius)];
    disp(file_title);

    % Solve grasping synthesis optimization problem
    [hand, object, opt_soln, opt_cost, if_solution] = mainSingleGrasping(hand, object, recon, os_pair, false, false, file_title);

    % Visualize and save results
    if if_solution
        visualizeOptimizationConfig(hand, object, opt_soln.X_sol, opt_soln.param);

        save(['../database/results/',file_title,'.mat']);
        savefig(['../database/results/',file_title,'.fig']);
    else
        disp('No solution obtained. Exit.');
    end
end
