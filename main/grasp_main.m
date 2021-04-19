% main script to perform a grasp
clc;
% adding paths to the workspace
setup_path;
% generate problem configuration
setup_problem_config;

%% Configuration of experiment
recon.hand_model = false; % reconstruct hand models
recon.object_model = true; % reconstruct object models
recon.rmap = false; % reconstruct reachability maps
recon.os = true; % reconstruct opposition space

%% Create Hand Models
if recon.hand_model || ~exist('hand','var')
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
type = 'comp'
switch type
    case 'cyl'
        Param.radius = 14;
        Param.height = 30;
        Param.roll = 0;
        Param.pitch = pi/2;
        Param.yaw = 0;
        % bad solution: roll = pi/8, pitch = pi/6, yaw = 0, os={[2,4],[3,4]}
        Param.quat = quaternion([Param.yaw,Param.pitch,Param.roll],'euler', 'ZYX','frame');
        Param.transl = [0;0;-60]; % translation
        object = cylinderObj(Param);

        mySGplotHand(hand);
        plotCylinder(object,false);
    case 'comp'
        Param.radius = 12;
        Param.height = 20;
        Param.roll = 0;
        Param.pitch = -pi/2;
        Param.yaw = 0;
        % bad solution: roll = pi/8, pitch = pi/6, yaw = 0, os={[2,4],[3,4]}
        Param.quat = quaternion([Param.yaw,Param.pitch,Param.roll],'euler', 'ZYX','frame');
        Param.transl = [0;0;-100]; % translation
        
        Param.sphereCenter = [0,0,20;15,0,20;-15,0,20];
        Param.sphereRadius = [12,12,12];
        object = compObj(Param);
        mySGplotHand(hand);
        plotCompObject(object,false);
end
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

osList = {{[1,4],[2,4]}};%,...
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

for i = 1:numel(osList)
    fprintf('Experiment: %d\n', i);
    
    os_pair = osList{i};

    file_title = ['single_grasp_'...
        '_F',num2str(os_pair{1}(1)),'L',num2str(os_pair{1}(2)),...
        '_F',num2str(os_pair{2}(1)),'L',num2str(os_pair{2}(2)),...
        '_type_',object.type,'_r_',num2str(object.radius),'_h_',num2str(object.height)];
    disp(file_title);

    % Solve grasping synthesis optimization problem
    [hand, object, opt_soln, opt_cost, if_solution] = graspSingleCylinder(hand, object, recon, os_pair, false, false, file_title);

    % Visualize and save results
    if if_solution
        visualizeOptimizationConfig(hand, object, opt_soln.X_sol, opt_soln.param);
        
        save(['../database/results/',file_title,'.mat']);
        savefig(['../database/results/',file_title,'.fig']);
    else
        disp('No solution obtained. Exit.');
    end
end
