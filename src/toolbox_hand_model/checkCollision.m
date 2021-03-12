clc;
clear all;
close all;
init;
 
models = load('cleanModels.mat');
hand = models.hand;
fprintf('Loaded hand model\n');

%% hand randomization

nf = hand.n; % number of fingers
for f = 1:nf
    finger = hand.F{f};
    nl = finger.nlink; % number of links of f-th finger
    nd = finger.n; % number of dof for f-th finger (1 fewer than nl, as nl contains fingertip)
    q = zeros(1, nd);
    q_dim = cell(1,nd);
    for d = 1:nd
        if ~finger.active_joints(d) % if already used (inactive)
            fprintf('Joint %d is not active.\n', d);
            q_dim{d} = finger.q(d); % keep the current angle
        else
            q_dim{d} = finger.lb(d) + rand * (finger.ub(d) - finger.lb(d)); % take random position
        end
    end
    q_mesh = cell(1,nd); % output meshgrid, each cell is value of one dimension
    [q_mesh{:}] = ndgrid(q_dim{:});
    q_test = zeros(1,nd); % current q to sample
    for d = 1:nd % assign each joint value
        q_test(d) = q_mesh{d};
    end
    finger = moveFinger(finger, q_test, 'sym'); % link information is updated in 'moveFinger' function
    hand.F{f} = finger;
end

%% finger sampling 

N = 50; % sampling number

%d_obj = zeros(nf, nl); % distance of each link to the object 
%d_link = NaN(nf, nf, nl, nl); % distance of each link to one another
%p_sampled = zeros(nf, nl, N, 3);

for f = 1:nf
   finger = hand.F{f};
   nl = finger.nlink; % number of links of f-th finger
   for l = 1:nl
       link = finger.Link{l};
       if ~link.is_real
           continue;
       end
       x1 = finger.Link{l}.p; % base point of joint 
       x2 = finger.Link{l+1}.p; % end point of joint
       %p_sampled{f,l} = sampleseg(x1,x2,N);
       p_sampled(f,l,:,:) = sampleseg(x1, x2, N); % sample the joint
   end
end

%% object sampling
Mx = 9;
My = 7;
Mz = 6;
M = Mx * My * Mz;

% create equally spaced points along the x, y and z axis
ox = linspace(-80, 80, Mx);
oy = linspace(0, 120, My);
oz = linspace(-100, 0, Mz);
obj_list = cell(1,M);

i = 1;
for x = 1:Mx
    a = ox(x);
   for y = 1:My
       b = oy(y);
       for z = 1:Mz
           c = oz(z);
           H = [1,0,0,a;...
                0,1,0,b;...
                0,0,1,c;...
                0,0,0,1];
           obj_list{i} = mySGsphere(H, 10); % Construct sphere objects;
           i = i + 1;
       end
   end
end

fprintf("Sampled %d objects\n", M);



%% collision

epsilon = 10e-30;
link_r = hand.hand_radius; 
obj_r = obj_list{1}.radius; % assuming constant radius across objects

n_obj_coll = 0;
n_link_coll = 0;

for f = 1:nf
    finger = hand.F{f};
    nl = finger.nlink; % number of links of f-th finger
    for l = 1:nl
        link = finger.Link{l};
        if ~link.is_real % check if link is real
            d_obj(f,l) = inf;
            continue;
        end
        
        % link/object collision
        for o = 1:M
            obj = obj_list{o};
            p_sampled_temp(:,:) = p_sampled(f,l,:,:);
            d = min(pdist2(p_sampled_temp, obj.center', 'euclidean'));
            d_obj(f,l) = d - link_r - obj_r;
            
            if(d-link_r-obj_r < epsilon)
                n_obj_coll = n_obj_coll + 1;
                obj_coll_list{n_obj_coll} = {obj,[f,l]};
            end
        end
        
        % distance to line - LEGACY
    %     link_dist = link.symbolic.link_dist; % symbolic distance
    %     c = (link_dist - link_r - obj_r); 
    %     c = subs(c, finger.q_sym, finger.q.'); % subtitute q values of finger
    %     dist(l) = double(subs(c, symvar(c), obj.center')); % subtitute center position of object
    
        % link/link collision
        for f2 = f:nf
            for l2 = l:nl
                link2 = finger.Link{l2};
                
                if ~link2.is_real % check if link is real 
                    continue;
                end
                
                if f == f2 % check if both links are equal or concecutive
                    if l == l2 || l + 1 == l2 %|| l + 2 == l2 
                        continue;
                    end
                end
                p_sampled_temp(:,:) = p_sampled(f,l,:,:);
                p_sampled_temp2(:,:) = p_sampled(f2,l2,:,:);
                d = min(pdist2(p_sampled_temp, p_sampled_temp2), [], 'all'); % get euclidean distance between two links
                d_link(f,f2,l,l2) = d - 2 * link_r;
                
                if(d-2*link_r < epsilon)
                    n_link_coll = n_link_coll + 1;
                    link_coll{n_link_coll} = [f, f2, l, l2];
                    fprintf("Link collision between finger %d and %d, link %d and %d \n", f, f2, l, l2)
                end
            end
        end
    end
end
% ocollision = d_obj < epsilon;
% if any(ocollision, 'all') 
%     [a, b] = find(ocollision);
%     fprintf("Object collision on finger %d, joint %d\n",a, b-1);
% end
% fprintf("\n");

%% Plotting
plot_all_objects = false;
plot_colliding_objects = false;

close all;
mySGplotHand(hand);
if(plot_all_objects)
    hold on;
    for i = 1:M
        obj = obj_list{i};
        SGplotSolid(obj);
        hold on;
    end
elseif(plot_colliding_objects)
    hold on;
    for i = 1:n_obj_coll
        obj = obj_coll_list{i}{1};
        SGplotSolid(obj);
        hold on;
    end
end
%% Sampling function
function [out] = sampleseg(x1, x2, N)
% this function samples the segment formed by the points [x1, x2] into N
% points. The output is a N x 3 matrix containing the N points.

    x = linspace(x1(1), x2(1), N);
    y = linspace(x1(2), x2(2), N);
    z = linspace(x1(3), x2(3), N);
    out = [x; y; z]';
end
