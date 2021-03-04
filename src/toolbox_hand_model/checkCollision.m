clc;
clear all;
close all;
init;

models = load('cleanModels.mat');
hand = models.hand;

% take the index finger
finger = hand.F{2};
n = hand.n; % number of fingers
nl = finger.nlink; % number of links of f-th finger
nd = finger.n; % number of dof for f-th finger (1 fewer than nl, as nl contains fingertip)

q = zeros(1, nd);
q_dim = cell(1,nd);
for d = 1:nd
    if ~finger.active_joints(d) % if already used (inactive)
        fprintf('Joint %d is not active.\n', d);
        q_dim{d} = finger.q(d); % keep the current angle
    else
        q_dim{d} = finger.lb(d); % just take lower bound
    end
end
q_mesh = cell(1,nd); % output meshgrid, each cell is value of one dimension
[q_mesh{:}] = ndgrid(q_dim{:});

%num_q = numel(q_mesh{1}); % number of all q-values to sample
q_test = zeros(1,nd); % current q to sample
for d = 1:nd % assign each joint value
    q_test(d) = q_mesh{d};
end

finger = moveFinger(finger, q_test, 'sym'); % link information is updated in 'moveFinger' function

H = [1,0,0,30;...
     0,1,0,130;...
     0,0,1,40;...
     0,0,0,1];
obj = mySGsphere(H, 10); % Construct sphere object
hand.F{2} = finger;

%% Collision testing
epsilon = 10e-30;
obj_r = 10;
link_r = hand.hand_radius;

dist = zeros(n,nl);

for f = 1:n
    finger = hand.F{f};
    for l = 1:nl
        link = finger.Link{l};
        if ~link.is_real
            dist(f,l) = inf;
            continue;
        end

        % distance to segment
        d = distance2Link(obj.center, link);
        dist(f,l) = d - link_r - obj_r;

        % distance to line - LEGACY
    %     link_dist = link.symbolic.link_dist; % symbolic distance
    %     c = (link_dist - link_r - obj_r); 
    %     c = subs(c, finger.q_sym, finger.q.'); % subtitute q values of finger
    %     dist(l) = double(subs(c, symvar(c), obj.center')); % subtitute center position of object
    end
end
collision = dist < epsilon;
if any(collision, 'all') 
    [a, b] = find(collision);
    fprintf("collision on finger %d, joint %d\n",a, b-1);
end

%% Plotting
mySGplotHand(hand);
hold on;
SGplotSolid(obj);
