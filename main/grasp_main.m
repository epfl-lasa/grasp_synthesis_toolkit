% main script to perform a grasp
clc;
% adding paths to the workspace
setup_path;
% generate problem configuration
setup_problem_config;

%% Configuration of experiment
recon.hand_model = true; % reconstruct hand models
recon.object_model = true; % reconstruct object models
recon.rmap = true; % reconstruct reachability maps
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

