% EXPERIMENT -ANALYSIS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clear all;
close all;
%% load the data as tables
T_opt = readtable('../database/robot_experiment/analysis/experiment_results_optim.csv');
T_exp = readtable('../database/robot_experiment/analysis/experiment_results_exp.csv');

% unused: compute bias separately
% hand_center = [0.931719,-1.142540, 1.365805]; % hand center in meters

hand_ctr_sim = [0,-29.4975,0];   % hand center in Matlab [mm]

rot_x = 1000*[1,0,0;0,0,1;0,-1,0];  % transformation  MCS to Simulation
                                    % (convert [m] to [mm])
% MCS -> S 
% y in S = z in MCS
% z in S = -y in MCS

%%% load logfiles
nb_experience = 21;

exp_data  = T_exp(1:nb_experience,:);
obj_ctr_mc = table2array(exp_data(:,{'x','y','z'}));

opt_data = T_opt(1:nb_experience,:);
obj_ctr_sim = table2array(opt_data(:,{'x','y','z'}));
obj_ctr_sim = obj_ctr_sim - hand_ctr_sim;

%%% transform motion capture data to simulation frame

% object center according to mc frame
obj_ctr = (rot_x * (obj_ctr_mc.')).'; % size ( N x 3 )

% compute the offset 
offset = mean(obj_ctr_sim,1) - mean(obj_ctr,1);

% add this bias to the obj_ctr;
obj_ctr = obj_ctr + offset; % correct for the wrong origin

% compute the array for c1 and c2 (for orientation)
c1_array = table2array(exp_data(:,{'c1_1','c1_2','c1_3'}));
c2_array = table2array(exp_data(:,{'c2_1','c2_2','c2_3'}));

c1_array = (rot_x * c1_array.').' + offset;
c2_array = (rot_x * c2_array.').' + offset;

% retrieve orientations
axis_sim = table2array(opt_data(:,{'quat2','quat3','quat4'}));
%%% analyze the data
nb_experience = 21;
is_sph = [ones(1,9),zeros(1,12)];
is_cyl = [zeros(1,9),ones(1,8),zeros(1,4)];
is_comp = [zeros(1,17),ones(1,4)];

% initialize all arrays
joint_angles = {'q1','q2','q3','q4','q5','q6','q7','q8','q9','q10','q11','q12','q13','q14','q15','q16'};

% construct an array containting info about contacting fingers
finger_data = table2array(exp_data(:,{'finger1','finger2'}));

% compute joint angle variations only for the active fingers
% note that thumb has index q13 - q16
F = [{'q13','q14','q15','q16'};.... % F1
    {'q1','q2','q3','q4'};...       % F2
    {'q5','q6','q7','q8'};...       % F3    
    {'q9','q10','q11','q12'}];...    % F4
    
% center position of all objects
sph_ctr_pos_err = []; 
sph_ctr_dist = [];
sph_err_JA = [];
sph_err_axis = [];

cyl_ctr_pos_err = [];
cyl_ctr_dist = [];
cyl_err_JA = [];
cyl_err_axis = [];

comp_ctr_pos_err = [];
comp_ctr_dist = [];
comp_err_JA = [];
comp_err_axis = [];


center_opt_arr = [];
center_exp_arr = [];

hand_center_predictions = [];
% loop over all 20 experiences
for i=1:nb_experience
    % center position in motion capture frame
    oc_mc = obj_ctr(i,:);      % obj_ctr of recorded data
    oc_sim = obj_ctr_sim(i,:); % obj_ctr of simulation
    
    pos_diff = oc_sim - oc_mc; % position difference simulation w.r.t. MC
    d_pos_diff = norm(pos_diff,2);
    
    % retrieve joing angles
    finger_pair = finger_data(i,:);
    finger_pair = finger_pair(finger_pair ~= 0); % remove palm
    if length(finger_pair) == 2
        grasp_joint_angles = [F(finger_pair(1),:), F(finger_pair(2),:)];  
    else
        grasp_joint_angles = [F(finger_pair,:)];
    end
    joint_angles_rec = table2array(exp_data(i,grasp_joint_angles));
    joint_angles_sim = table2array(opt_data(i,grasp_joint_angles));
    
    % standard deviation in JA error
    err_JA = std(joint_angles_rec - joint_angles_sim);    
    
    % computation of the orientation (only for non-spherical objects)
    if ~is_sph(i)
        c1 = c1_array(i,:).';
        c2 = c2_array(i,:).';
        
        ax_mc = (c1-c2)/norm(c1-c2, 2); % object axis of recorded data
        
        % object axis of simulation (normalize it)
        ax_sim = axis_sim(i,:).' / norm(axis_sim(i,:),2); 

        axis_err = 1 - abs(dot(ax_mc,ax_sim));
    end
    
    if is_sph(i)
        sph_ctr_pos_err(end+1,:) = pos_diff;
        sph_ctr_dist(end+1) = d_pos_diff;
        sph_err_JA(end+1) = err_JA;
    end
    
    if is_cyl(i)
        cyl_ctr_pos_err(end+1,:) = pos_diff;
        cyl_ctr_dist(end+1) = d_pos_diff;
        cyl_err_JA(end+1) = err_JA;
        cyl_err_axis(end+1) = axis_err;
    end
    
    if is_comp(i)
       comp_ctr_pos_err(end+1,:) = pos_diff;
       comp_ctr_dist(end+1) = d_pos_diff;
       comp_err_JA(end+1) = err_JA;
       comp_err_axis(end+1) = axis_err;
    end    
    
end


% PLOT POSITION ERROR ALONG EACH AXIS
figure
ax = gca;
ax.FontSize = 20; 
subplot(311),scatter(1:size(sph_ctr_dist,2),sph_ctr_pos_err(:,1),'*');
xlim([1,21])
hold on
scatter(10:9+size(cyl_ctr_dist,2),cyl_ctr_pos_err(:,2),'*');
scatter(18:17+size(comp_ctr_dist,2),comp_ctr_pos_err(:,2),'*');
ylabel('$e_x$','Interpreter','latex','Fontsize',20);
hold on
subplot(312),scatter(1:size(sph_ctr_dist,2),sph_ctr_pos_err(:,2),'*');
xlim([1,21])
hold on
scatter(10:9+size(cyl_ctr_dist,2),cyl_ctr_pos_err(:,2),'*');
scatter(18:17+size(comp_ctr_dist,2),comp_ctr_pos_err(:,2),'*');
ylabel('$e_y$','Interpreter','latex','Fontsize',20);
subplot(313),scatter(1:size(sph_ctr_dist,2),sph_ctr_pos_err(:,3),'*');
hold on
subplot(313),scatter(10:9+size(cyl_ctr_dist,2),cyl_ctr_pos_err(:,3),'*');
subplot(313),scatter(18:17+size(comp_ctr_dist,2),comp_ctr_pos_err(:,3),'*');
xlim([1,21])
ylabel('$e_z$','Interpreter','latex','Fontsize',20);
xlabel('Experience','Fontsize',20)
sgtitle('Position error','Fontsize',24)

% DISTANCE BOXPLOTS
figure
ax = gca;
ax.FontSize = 20; 
subplot(1,3,1), boxplot(sph_ctr_dist)
ylim([0,100])
ylabel('Distance to simulation [mm]','Fontsize',16')
xlabel('Sphere objects','Fontsize',16')
subplot(1,3,2), boxplot(cyl_ctr_dist)
ylim([0,100])
xlabel('Cylinder objects','Fontsize',16')
subplot(1,3,3), boxplot(comp_ctr_dist)
ylim([0,100])
xlabel('Composite objects','Fontsize',16')
saveas(gcf, '../results/distance_deviation.fig')
saveas(gcf, '../results/distance_deviation.png')

% JOINT ANGLE BOXPLOTS
figure
ax = gca;
ax.FontSize = 16; 

subplot(1,3,1), boxplot(sph_err_JA)
ylim([0,0.1])
ylabel('Joint angle deviation [rad]','Fontsize',16)
xlabel('Sphere objects','Fontsize',16)
subplot(1,3,2), boxplot(cyl_err_JA)
ylim([0,0.1])
xlabel('Cylinder objects','Fontsize',16)
subplot(1,3,3), boxplot(comp_err_JA)
ylim([0,0.1])
xlabel('Composite objects','Fontsize',16')
%sgtitle('Standard deviation in joint angles', 'FontWeight','bold','Fontsize',20)

saveas(gcf, '../results/joint_angle_deviation.fig')
saveas(gcf, '../results/joint_angle_deviation.png')

% ORIENTATION ANGLE BOXPLOTS
figure
ylabel('[rad]')
subplot(1,2,1), boxplot(cyl_err_axis)
ylim([0,1])
xlabel('Cylinder objects','Fontsize',16')
ylabel('Orientation deviation','Fontsize',16')
subplot(1,2,2), boxplot(comp_err_axis)
ylim([0,1])
xlabel('Composite objects','Fontsize',16')
saveas(gcf, '../results/orientation_deviation.fig')
saveas(gcf, '../results/orientation_deviation.png')
% 
% % plot all distances
% % figure
% % subplot(1,3,1), plot(sph_ctr_pos)
% % subplot(1,3,2), plot(cyl_ctr_pos)
% % subplot(1,3,3), plot(comp_ctr_pos)
% 
% % figure
% % subplot(1,3,1),plot(center_opt(:,1))
% % hold on
% % plot(center_exp(:,1))
% % xlim([1 21])
% % xlabel('Experience')
% % ylabel('Distance [mm]')
% % legend('OPT','EXP')
% % title('Position estimation: x-axis')
% % 
% % subplot(1,3,2),plot(center_opt(:,2))
% % hold on
% % plot(center_exp(:,2))
% % xlim([1 21])
% % xlabel('Experience')
% % ylabel('Distance [mm]')
% % legend('OPT','EXP')
% % title('Position estimation: y-axis')
% % 
% % subplot(1,3,3),plot(center_opt(:,3))
% % hold on
% % plot(center_exp(:,3))
% % xlim([1 21])
% % xlabel('Experience')
% % ylabel('Distance [mm]')
% % legend('OPT','EXP')
% % title('Position estimation: z-axis')
