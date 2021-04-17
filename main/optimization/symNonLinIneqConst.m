% Obtain the symbolic form of nonlinear inequality constraints
function [c, c_grad, param, ht_c, ht_c_grad] = symNonLinIneqConst(hand, param)
    os_info = param.os.os_info;
    
    % parameters
    ncp = param.ncp;                % number of contact points (palm incl.)
    obj_r = param.obj.radius;       % object radius
    link_r = param.hand_radius;     % finger radius of the hand
   
    % symbolic variables
    X_key = param.X_key;
    obj_axpt = param.obj.sym.axisPtArray;   % equidistant points along axis
    obj_cnt = param.obj.sym.ctr;            % object center

    dict_q = param.dict_q; % dictionary of ALL symbolic joint angles: q11,q12,...
    
    fprintf('\nConstructing Nonl. Inequality Constraints: \n');
    c = sym([]); % to save the symbolic form
    c_idx = []; % to save idx of different constraints
    c_name = {};
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 1: Collision Avoidance
    % Collision between links (this link and all father links) and object
    % dist. between all previous links & object larger than object radius
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Notice that this only contains all previous links (incl. virtual
    % ones), since the current link is in contact, and this constraint is
    % formulated as nonl equality constraint.
    fprintf('* [1/5] Collision avoidance (object vs. active finger links): ');
    for i = 1:ncp
        if ~all(os_info{i})
            % collision with palm is guaranteed in 'Collision avoidance (object vs. palm)' 
            continue;
        else
            [idx_f, idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger and link
            finger = hand.F{idx_f};
            nq_pre = idx_l-1;
            if (idx_l > finger.n)
                warining("Nonlinear constriaint definition: link index of fingertip");
            end
            for j = 1:nq_pre % Iterate over all links, including link in contact ('idx_l')
                link = finger.Link{j};
                % define constraints only if the finger link is real
                if link.is_real
                    % [Cylinder implementation]
                    nAxpt = size(obj_axpt,2); % obj_axpt is (3 x n_points)
                    eta = 1.5;
                    nLinkPoint = ceil(link.L/(2*hand.hand_radius*sqrt(eta^2-1)));
                    pointDist = 1/nLinkPoint;
                    del = pointDist/2 + [0:nLinkPoint-1].*pointDist;
                    for k=1:nAxpt
                        link_pos_this = link.symbolic.HT_this(1:3,4);
                        link_pos_next = link.symbolic.HT_next(1:3,4);
                        delPos = link_pos_next - link_pos_this;
                        for l=1:nLinkPoint
                            link_pos = link_pos_this + del(l)*delPos;
                            % increase the radius by a factor eta
                            c_ij = link_r*eta + obj_r - norm(link_pos-obj_axpt(:,k),2);
                            c(end+1) = c_ij;
                        end
                    end
                end
            end
            % define collision avoidance contacting link
            link = finger.Link{idx_l};
            %%% assume that the maximal penetration is 10% of the hand
            %%% radius
            d_max = hand.hand_radius * cos(asin(0.7));
            nLinkPoint = ceil(link.L/d_max);
            pointDist = 1/nLinkPoint;
            del = pointDist/2 + [0:nLinkPoint-1].*pointDist;
            for k=1:nAxpt
                link_pos_this = link.symbolic.HT_this(1:3,4);
                link_pos_next = link.symbolic.HT_next(1:3,4);
                delPos = link_pos_next - link_pos_this;
                for l=1:nLinkPoint
                    link_pos = link_pos_this + del(l)*delPos;
                    % take the true link radius here
                    c_ij = link_r + obj_r - norm(link_pos-obj_axpt(:,k),2);
                    c(end+1) = c_ij;
                end
            end
        end
    end
    c_idx(end+1) = numel(c);
    c_name{end+1} = 'Collision avoidance (object vs. links)';
    fprintf('%d\n', c_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 2: Collision Avoidance
    % Collision between object and included fingers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* [2/5] Collision avoidance (object vs. included fingers): ');
    % included finger: finger that between two active fingers, e.g. the middle finger is included if using index finger and ring finger for grasping
    f_idx_list = zeros(1,ncp); % list of finger index
    for i = 1:ncp
        f_idx_list(i) = os_info{i}(1);
    end
    inBetweenF = min(f_idx_list)+1:max(f_idx_list)-1; % indices of in-between fingers
  
    if ~isempty(inBetweenF) % in-between finger exists
        for f = 1:length(inBetweenF)
            finger = hand.F{inBetweenF(f)}; % same as collision avoidance: father-links vs. object
            for i = 1:finger.n-1
                link = finger.Link{i};
                nAxpt = size(obj_axpt,2);
                for k=1:nAxpt
                    link_pos_this = link.symbolic.HT_this(1:3,4);
                    link_pos_next = link.symbolic.HT_next(1:3,4);
                    eta = 1.5;
                    nLinkPoint = ceil(link.L/(2*hand.hand_radius*sqrt(eta^2-1)));
                    pointDist = 1/nLinkPoint;
                    del = pointDist/2 + [0:nLinkPoint-1].*pointDist; % in [0,1]
                    delPos = link_pos_next - link_pos_this; % symbolic
                    
                    for j=1:nLinkPoint
                        link_pos = link_pos_this + del(j)*delPos;
                        c_ij = link_r + obj_r - norm(link_pos - obj_axpt(:,k),2) ;
                        c(end+1) = c_ij;
                    end
                end
            end
        end
    end
    
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (object vs. in-between fingers)';
    fprintf('%d\n', c_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 3: Collision Avoidance
    % Collision between object and palm (finger bases)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* [3/5] Collision avoidance (object vs. palm): ');
    
    % [TODO] implement that part!! (use axis points)
    %x1 = hand.F{1}.symbolic.base(1:3,4);
    %x2 = hand.F{end}.symbolic.base(1:3,4);
    % Line x1-x2 is the 'finger base line' that connects all finger bases
    % This detection is realized by calculating the distance between object
    % center and the finger base line
    %dist = norm(cross(obj_cnt-x1,obj_cnt-x2))/norm(x2-x1);
    palmPt = hand.P.points_inr(1,:).'; % point on the inner surface of the palm
    nAxpt = size(obj_axpt,2);
    palmNormal = hand.P.contact.symbolic.n;
    for k=1:nAxpt
        dist = palmNormal.' * (obj_axpt(:,k)-palmPt); % project the object on the normal
        c(end+1) = link_r + obj_r - dist;
    end
    
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (object vs. palm)';
    fprintf('%d\n', c_idx(end));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 4: Collision Avoidance
    % Collision between current link and links from other fingers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* [4/5] Collision avoidance (this link vs. other links): ');
    all_collision_pairs = {};
    for i = 1:ncp
        if ~all(os_info{i})
            continue; % palm does not move, will not collide with other fingers
        else
            [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger and link in the finger
            finger = hand.F{idx_f};
            nq_pre = min([idx_l, finger.nlink-1]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link). max(finger.n) = 4

            for j = 1:nq_pre % Iterate over the current link + all father-links on this finger
                link = finger.Link{j}; % link j, The link being checked at this moment
                if ~link.is_real
                    continue;
                end
                x1 = link.symbolic.HT_this(1:3,4); % end points of current link
                x2 = link.symbolic.HT_next(1:3,4);
                coll = link.collList; % retrieve the links that collide with the current one

                for k = 1:numel(coll) % link k collides with current link
                    [k_f,k_l] = deal(coll{k}(1),coll{k}(2));

                    if k_f == idx_f % skip link on the same finger
                        continue;
                    end

                    link_coll = hand.F{k_f}.Link{k_l}; % link to collide with current link
                    if ~link_coll.is_real
                        continue;
                    end
                    
                    % filter out repetitive collision pairs 
                    already_exist = false; % if the current pair of collision already exists in the collision pair
                    if ~isempty(all_collision_pairs)
                        for p = 1:numel(all_collision_pairs)
                            temp_pair = all_collision_pairs{p}; % should be [2*2]: [finger_1, link_1; finger_2, link_2]
                            if ismember([idx_f,idx_l],temp_pair,'rows') && ismember([k_f,k_l],temp_pair,'rows')
                                already_exist = true;
                                break;
                            end   
                        end
                    end
                    if already_exist
                        continue;
                    else
                        all_collision_pairs{end+1} = [idx_f,idx_l;k_f,k_l];
                    end
                    y1 = link_coll.symbolic.HT_this(1:3,4); % end points of collided link
                    y2 = link_coll.symbolic.HT_next(1:3,4);
                    
                    % Here is the min. distance between two 3d line segments.
                    % Not min. distance between two 3d lines!
                    dist = distanceBTW2Lines(x1,x2,y1,y2,5); % minimum distance between two links (line segments) N=5: 5 sampling points
                    % All virtual father links will result in a 1*1 dist.
                    %%% Substitute all parameters (qi) that are not in X_key with hand joint values
                    symvars = symvar(dist);
                    subKeys = symvars(~ismember(symvars,X_key)); % symvars that belong to dict_q but not X_key, need to be substituted
                    if ~isempty(subKeys)
                        subValues = hand.q(ismember(dict_q,subKeys)); % subs them using hand default joint angle values
                        dist = subs(dist, subKeys, subValues.');
                    end
                    try
                        c(end+1 : end+numel(dist)) = 2*link_r - dist;
                    catch
                        disp('I don`t know.');
                    end
                end
                    
            end
        end
    end
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (this link vs. other links)';
    fprintf('%d\n', c_idx(end));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 5: Collision Avoidance
    % Collision between target object and already grasped objects
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    grasped_objects = param.grasped_objects;
    if ~isempty(grasped_objects)
        fprintf('* [5/5] Collision avoidance (target object and object in grasp): ');
        nobj = numel(grasped_objects);
        for i = 1:nobj
            obj_i = grasped_objects{i};
            oc_i = obj_i.center;
            radius_i = obj_i.radius;
            c(end+1) = (obj_r+radius_i) - norm(obj_cnt(:)-oc_i(:)); % distance between centers of target object and anyone of the grasped object is larger than the sum of their radius
        end
        c_idx(end+1) = numel(c)-sum(c_idx);
        c_name{end+1} = 'Collision avoidance (target object vs. grasped objects)';
        fprintf('%d\n', c_idx(end));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Save symbolic form of constraints
    param.c_idx = c_idx;
    param.c_name = c_name;
    
    matlabFunction(c,'File','../database/symbolic_functions/nonl_c','Vars',X_key,'Optimize',false);

    %%% Calculate gradient Gradient follows the format. Notice that this is the TRANSPOSE of Jacobian!
    % [dc1/dx1, dc2/dx1;...
    %  dc1/dx2, dc2/dx2];
    c_grad = transpose(jacobian(c, X_key));
    
    matlabFunction(c_grad,'File','../database/symbolic_functions/nonl_c_grad','Vars',X_key,'Optimize',false);
    fprintf('  Total num. of nonl. inequality constraints: %d\n', numel(c));
    
    if nargout > 3 % create function handles
        ht_c = matlabFunction(c,'Vars',X_key);
        ht_c_grad = matlabFunction(c_grad,'Vars',X_key);
    end
end