% EXPERIMENT -ANALYSIS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clear all;
close all;
%% load the data as tables
T_opt = readtable('../results/experiment_results_optim.csv');
T_exp = readtable('../results/experiment_results_exp.csv');

% unused: compute bias separately
% hand_center = [0.931719,-1.142540, 1.365805]; % hand center in meters

hand_center_opt = [0,-29.4975,0];   % hand center in Matlab [mm]

rot_x = 1000*[1,0,0;0,0,1;0,-1,0];  % matrix to rotate from MC-frame to Matlab frame
                                    % (converts also to [mm]


%%% compute the bias

% retrieve data of object centers for the first 2 experiments
exp_data  = T_exp(1:2,:);
center_mc = table2array(exp_data(1:2,{'x','y','z'}));

opt_data = T_opt(1:2,:);
center_opt = table2array(opt_data(1:2,{'x','y','z'}));

% transform to simulation frame
rel_center_mc = (rot_x * center_mc.').';

% compute the bias
bias =  mean(center_opt - rel_center_mc,1);


%%% analyze the data
nb_experience = 19
is_sph = [ones(1,7),zeros(1,12)];
is_cyl = [zeros(1,7),ones(1,8),zeros(1,4)];
is_comp = [zeros(1,15),ones(1,4)];
comp_distance = ones(1,21); % compute distance on all the samples

% initialize all arrays
pos_variables = {'x','y','z'};
joint_angles = {'q1','q2','q3','q4','q5','q6','q7','q8','q9','q10','q11','q12','q13','q14','q15','q16'};
% compute joint angle variations only for the active fingers
% note that thumb has index q13 - q16
F = [{'q13','q14','q15','q16'};.... % F1
    {'q1','q2','q3','q4'};...       % F2
    {'q5','q6','q7','q8'};...       % F3    
    {'q9','q10','q11','q12'}];...    % F4
    

% center position of all objects
sph_ctr_pos = []; 
sph_ctr_dist = [];
sph_err_JA = [];
sph_err_axis = [];

cyl_ctr_pos = [];
cyl_ctr_dist = [];
cyl_err_JA = [];
cyl_err_axis = [];

comp_ctr_pos = [];
comp_ctr_dist = [];
comp_err_JA = [];
comp_err_axis = [];


center_opt = [];
center_exp = [];

% loop over all 20 experiences
for i=1:nb_experience
    % center position in motion capture frame
    exp_data  = T_exp(i,:);
    pos_mc = table2array(exp_data(:,pos_variables)); % MC center position
    pos = (rot_x * pos_mc.').' + bias;     % convert to simulation frame
    
    % center position in optimization (in [m])
    opt_data = T_opt(i,:);
    pos_sim = table2array(opt_data(:,pos_variables))-hand_center_opt;
    
    
    %%% joint angle comparison
    fingers = table2array(exp_data(:,{'finger1','finger2'}));
    grasp_joint_angles = [];
    for j=1:2               % extract joint angle values of active fingers
        if fingers(j)~=0
            grasp_joint_angles = [grasp_joint_angles, F(fingers(j),:)];
        end
    end
    
    % compute standard deviation in active joint angles
    robot_JA = table2array(exp_data(:,grasp_joint_angles));
    sim_JA = table2array(opt_data(:,grasp_joint_angles));
    err_JA = std(robot_JA - sim_JA);
    
    
    
    %%% computation of the orientation
    c1 = table2array(exp_data(:,{'c1_1','c1_2','c1_3'}))';
    c1 = (rot_x * c1) + bias';
    c2 = table2array(exp_data(:,{'c2_1','c2_2','c2_3'}))';
    c2 = (rot_x * c2) + bias';
    
    axis_exp = (c1-c2)/norm(c1-c2, 2);
    
    axis_sim = table2array(opt_data(:,{'quat2','quat3','quat4'}))';
    
    axis_err = 1 - abs(dot(axis_exp,axis_sim));
    
    if is_sph(i)
        sph_ctr_pos(end+1,:) = pos - pos_sim;
        sph_ctr_dist(end+1) = norm(pos_sim,2);
        sph_err_JA(end+1) = err_JA;
    end
    
    if is_cyl(i)
        cyl_ctr_pos(end+1,:) = pos - pos_sim;
        cyl_ctr_dist(end+1) = norm(pos - pos_sim,2);
        cyl_err_JA(end+1) = err_JA;
        cyl_err_axis(end+1) = axis_err;
    end
    
    if is_comp(i)
       comp_ctr_pos(end+1,:) = pos - pos_sim;
       comp_ctr_dist(end+1) = norm(pos - pos_sim,2);
       comp_err_JA(end+1) = err_JA;
       comp_err_axis(end+1) = axis_err;
    end    
    
    center_opt(end+1,:) = pos_sim;
    center_exp(end+1,:) = pos;
end


% DISTANCE BOXPLOTS
figure
subplot(1,3,1), boxplot(sph_ctr_dist)
ylim([0,150])
ylabel('Distance to simulation [mm]')
xlabel('Sphere objects')
subplot(1,3,2), boxplot(cyl_ctr_dist)
ylim([0,150])
xlabel('Cylinder objects')
subplot(1,3,3), boxplot(comp_ctr_dist)
ylim([0,150])
xlabel('Composite objects')
sgtitle('Distance deviations compared to simulation', 'FontWeight','bold')
saveas(gcf, '../results/distance_deviation.fig')
saveas(gcf, '../results/distance_deviation.png')

% JOINT ANGLE BOXPLOTS
figure
subplot(1,3,1), boxplot(sph_err_JA)
ylim([0,0.1])

ylabel('[rad]')
xlabel('Sphere objects')
subplot(1,3,2), boxplot(cyl_err_JA)
ylim([0,0.1])
xlabel('Cylinder objects')
subplot(1,3,3), boxplot(comp_err_JA)
ylim([0,0.1])
xlabel('Composite objects')
sgtitle('Standard deviation in joint angles', 'FontWeight','bold')

saveas(gcf, '../results/joint_angle_deviation.fig')
saveas(gcf, '../results/joint_angle_deviation.png')

% ORIENTATION ANGLE BOXPLOTS
figure
ylabel('[rad]')
xlabel('Sphere objects')
subplot(1,2,1), boxplot(cyl_err_axis)
ylim([0,1])
xlabel('Cylinder objects')
subplot(1,2,2), boxplot(comp_err_axis)
ylim([0,1])
xlabel('Composite objects')
sgtitle('Deviation in orientation', 'FontWeight','bold')
saveas(gcf, '../results/orientation_deviation.fig')
saveas(gcf, '../results/orientation_deviation.png')

% plot all distances
% figure
% subplot(1,3,1), plot(sph_ctr_pos)
% subplot(1,3,2), plot(cyl_ctr_pos)
% subplot(1,3,3), plot(comp_ctr_pos)

% figure
% subplot(1,3,1),plot(center_opt(:,1))
% hold on
% plot(center_exp(:,1))
% xlim([1 21])
% xlabel('Experience')
% ylabel('Distance [mm]')
% legend('OPT','EXP')
% title('Position estimation: x-axis')
% 
% subplot(1,3,2),plot(center_opt(:,2))
% hold on
% plot(center_exp(:,2))
% xlim([1 21])
% xlabel('Experience')
% ylabel('Distance [mm]')
% legend('OPT','EXP')
% title('Position estimation: y-axis')
% 
% subplot(1,3,3),plot(center_opt(:,3))
% hold on
% plot(center_exp(:,3))
% xlim([1 21])
% xlabel('Experience')
% ylabel('Distance [mm]')
% legend('OPT','EXP')
% title('Position estimation: z-axis')
