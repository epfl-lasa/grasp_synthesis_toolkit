% Obtain the symbolic form of nonlinear inequality constraints
function [c, c_grad, param, ht_c, ht_c_grad] = compNonLinIneqConst(hand, param)
    os_info = param.os.os_info;
    
    % parameters
    ncp = param.ncp;                % number of contact points (palm incl.)
    obj_r = param.obj.radius;       % object radius
    link_r = param.hand_radius;     % finger radius of the hand
    obj = param.obj;
    % symbolic variables
    X_key = param.X_key;
    obj_axpt = param.obj.sym.axisPtArray;   % equidistant points along axis
    obj_cnt = param.obj.sym.ctr;            % object center

    dict_q = param.dict_q; % dictionary of ALL symbolic joint angles: q11,q12,...
    nCylinderAxis = 5;
    nb_axpt = size(obj_axpt,2);
    eta = 1.5;  % TODO change nqpre to idx_l, change lines of constraint definition
    is_eta = false;
    gamma = 0.5;
    
    palm_point = mean(hand.P.points_inr,1).';
    palm_normal = hand.P.contact.symbolic.n(:);
    
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
        [idx_f, idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger and link
        if (idx_f == 0)
            % collision with palm is guaranteed in 'Collision avoidance (object vs. palm)' 
            continue;
        end
        finger = hand.F{idx_f};
        if (idx_l > finger.n)
            warning("Nonlinear constraint definition: link index of fingertip");
        end
        if (is_eta)
            nq_pre = idx_l-1;
        else
            nq_pre = idx_l;
        end
        for j = 1:nq_pre % Iterate over all links, including link in contact ('idx_l')
            link = finger.Link{j};
            % define constraints only if the finger link is real
            if ~link.is_real
                continue;
            end
            
            if is_eta
                max_dist = 2*hand.hand_radius*sqrt(eta^2-1);
            else
                max_dist = 2*hand.hand_radius*cos(asin(gamma));
            end
            nb_link_pt = min([nCylinderAxis, ceil(link.L/max_dist)]);
            link_pt_dist = 1/nb_link_pt;
            link_pt_loc = link_pt_dist/2 + [0:nb_link_pt-1].*link_pt_dist;
            link_pos_this = link.symbolic.HT_this(1:3,4);
            link_pos_next = link.symbolic.HT_next(1:3,4);
            delta_pos = link_pos_next - link_pos_this;
            for m=1:nb_link_pt
                for n=1:nb_axpt
                    link_pt = link_pos_this + link_pt_loc(m)*delta_pos;
                    obj_pt = obj_axpt(:,n);
                    % squared distance for constraint
                    if is_eta
                        c_ij = (link_r + obj_r*eta)^2 - dot((link_pt-obj_pt),(link_pt - obj_pt));
                    else
                        c_ij = (link_r + obj_r)^2 - dot((link_pt-obj_pt),(link_pt - obj_pt));
                    end
                    c(end+1) = c_ij;
                end
            end
            % collsion with other spheres
            nSpheres = length(obj.sphereRadius);
            for m=1:nb_link_pt
                for n=1:nSpheres
                    link_pt = link_pos_this + link_pt_loc(m)*delta_pos;
                    sph_ctr = obj.sym.sphereCenter(:,n);
                    % take the true link radius here
                    if is_eta
                        c_ij = (link_r + obj_r*eta)^2 - dot((link_pt-obj_pt),(link_pt - obj_pt));
                    else
                        c_ij = (link_r + obj.sphereRadius(n))^2 - dot((link_pt-sph_ctr),(link_pt-sph_ctr));
                    end
                    c(end+1) = c_ij;
                end
            end
        end
        
        if (is_eta) % add the last link
            link = finger.Link{idx_l};
            % define constraints only if the finger link is real
            if ~link.is_real
                warning('Indicated contacting link is virtual\n');
                continue;
            end
            %max_dist = 2*hand.hand_radius*cos(asin(gamma));
            max_dist = 2*hand.hand_radius*sqrt(eta^2-1);
            nb_link_pt = min([nCylinderAxis, ceil(link.L/max_dist)]);
            link_pt_dist = 1/nb_link_pt;
            link_pt_loc = link_pt_dist/2 + [0:nb_link_pt-1].*link_pt_dist;
            link_pos_this = link.symbolic.HT_this(1:3,4);
            link_pos_next = link.symbolic.HT_next(1:3,4);
            delta_pos = link_pos_next - link_pos_this;
            for m=1:nb_link_pt
                for n=1:nb_axpt
                    link_pt = link_pos_this + link_pt_loc(m)*delta_pos;
                    obj_pt = obj_axpt(:,n);
                    % squared distance for constraint
                    c_ij = (link_r + obj_r*eta)^2 - dot((link_pt-obj_pt),(link_pt - obj_pt));
                    c(end+1) = c_ij;
                end
            end
            % collsion with other spheres
            nSpheres = length(obj.sphereRadius);
            for m=1:nb_link_pt
                for n=1:nSpheres
                    link_pt = link_pos_this + link_pt_loc(m)*delta_pos;
                    sph_ctr = obj.sym.sphereCenter(:,n);
                    % take the true link radius here
                    c_ij = (link_r + obj_r*eta)^2 - dot((link_pt-obj_pt),(link_pt - obj_pt));
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
    incl_fingers = min(f_idx_list)+1:max(f_idx_list)-1; % indices of in-between fingers
  
    if ~isempty(incl_fingers) % in-between finger exists
        for f = 1:length(incl_fingers)
            finger = hand.F{incl_fingers(f)}; % same as collision avoidance: father-links vs. object
            for i = 1:finger.n
                link = finger.Link{i};
                if ~link.is_real
                    continue;
                end
                link_pos_this = link.symbolic.HT_this(1:3,4);
                link_pos_next = link.symbolic.HT_next(1:3,4);
                if is_eta
                    max_dist = 2*hand.hand_radius*sqrt(eta^2-1);
                else
                    max_dist = 2*hand.hand_radius*cos(asin(gamma));
                end
                nb_link_pt = min([nCylinderAxis, ceil(link.L/max_dist)]);
                link_pt_dist = 1/nb_link_pt;
                link_pt_loc = link_pt_dist/2 + [0:nb_link_pt-1].*link_pt_dist; % in [0,1]
                delta_pos = link_pos_next - link_pos_this; % symbolic
                for m=1:nb_link_pt
                    for n=1:nb_axpt
                        link_pt = link_pos_this + link_pt_loc(m)*delta_pos;
                        obj_pt = obj_axpt(:,n);
                        if is_eta
                            c_ij = (link_r + obj_r*eta)^2 - dot((link_pt-obj_pt),(link_pt-obj_pt));
                        else
                        	c_ij = (link_r + obj_r)^2 - dot((link_pt-obj_pt),(link_pt-obj_pt));
                        end
                        c(end+1) = c_ij;
                    end
                end
                % collsion with other spheres
                nSpheres = length(obj.sphereRadius);
                for m=1:nb_link_pt
                    for n=1:nSpheres
                        link_pt = link_pos_this + link_pt_loc(m)*delta_pos;
                        sph_ctr = obj.sym.sphereCenter(:,n);
                        % take the true link radius here
                        if is_eta
                            c_ij = (link_r + obj.sphereRadius(n))^2 - dot((link_pt-sph_ctr),(link_pt-sph_ctr));
                        else
                            c_ij = (link_r + obj.sphereRadius(n))^2 - dot((link_pt-sph_ctr),(link_pt - sph_ctr));
                        end
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
    
    for k=1:nb_axpt
        dist = dot(palm_normal,(obj_axpt(:,k)-palm_point)); % project the object on the normal
        c(end+1) = (link_r + obj_r) - dist;
    end
    for k=1:length(obj.sphereRadius)
        dist = dot(palm_normal, (obj.sym.sphereCenter(:,k)-palm_point));
        c(end+1) = (link_r + obj.sphereRadius(k)) - dist;
    end
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (object vs. palm)';
    fprintf('%d\n', c_idx(end));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 4: Collision Avoidance
    % Collision between current link and links from other fingers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    fprintf('* Collision avoidance (this link vs. other links): ');
    all_collision_pairs = {};
    for i = 1:ncp
        [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2)); % This is the ith contacted link

        %%% This link is palm. Calculate collision between palm and links in its collision list.
        if ispalm(idx_f)
            try
                coll = hand.P.collList;
            catch
                fprintf('\nThe palm does not have collision list.\n');
                continue;
            end
            
            for k = 1:numel(coll) % link `k` collides with palm
                [k_f,k_l] = deal(coll{k}(1),coll{k}(2));
                
                % Check if current collision pair has already been detected previously
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
                
                if already_exist % if this distance has not been calculated before
                    continue;
                else
                    %%% Palm-Palm Collision
                    if ispalm(k_f)
                        error('Error: palm exists in the collision list of palm.');
                    %%% Palm-Link Collision
                    else
                        link_k = hand.F{k_f}.Link{k_l}; % link to collide with current link
                        if ~link_k.is_real
                            continue; % Skip virtual links
                        end
                        if k_l == find(hand.F{k_f}.idx_real_link,1) % if k_l is the first active link, skip it
                            continue;
                        end
                        x1 = link_k.symbolic.HT_this(1:3,4); % end points of collided link
                        x2 = link_k.symbolic.HT_next(1:3,4);

                        nSamples = min([nCylinderAxis, ceil(link_k.L/(link_k.radius*2))]);
                        dist2palm = sym(zeros(1,nSamples));
                        for d = 0:(nSamples-1)
                            xi = x1 + (x2-x1)*d/(nSamples-1);
                            dist2palm(d+1) = dot(palm_normal,xi-palm_point(:));
                        end

                        symvars = symvar(dist2palm);
                        subKeys = symvars(~ismember(symvars,X_key)); % symvars that belong to dict_q but not X_key, need to be substituted
                        if ~isempty(subKeys)
                            subValues = hand.q(ismember(dict_q,subKeys)); % subs them using hand default joint angle values
                            dist2palm = subs(dist2palm, subKeys, subValues.');
                        end
                        c(end+1 : end+numel(dist2palm)) = link_k.radius - dist2palm;
                        
                        all_collision_pairs{end+1} = [idx_f,idx_l; k_f,k_l]; % Add current collision pair to the detected list
                    end 
                end
            end

        %%% This link-in-contact is a normal finger link. Calculate collision between this link and links in its collision list.
        else
            finger = hand.F{idx_f};
            nq_pre = min([idx_l, finger.nlink-1]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link). max(finger.n) = 4

            for idx_p = 1:nq_pre % Iterate over the current link + all father-links on this finger. idx_p: idx of this preceded link
                link = finger.Link{idx_p}; % link j, The link being checked at this moment
                if ~link.is_real
                    continue;
                end

                try
                    coll = link.collList; % retrieve the links that collide with the current one
                catch
                    fprintf('  Finger: %d, Link: %d does not have collision list.\n', idx_f, j);
                    continue;
                end

                x1 = link.symbolic.HT_this(1:3,4); % end points of current link
                x2 = link.symbolic.HT_next(1:3,4);

                for k = 1:numel(coll) % link k collides with current link
                    [k_f,k_l] = deal(coll{k}(1),coll{k}(2));

                    if k_f == idx_f % skip link on the same finger
                        continue;
                    end

                    if abs(k_f-idx_f)>1 % skip non-adjacent fingers
                        continue;
                    end
                    
                    already_exist = false; % if the current pair of collision already exists in the collision pair
                    if ~isempty(all_collision_pairs)
                        for p = 1:numel(all_collision_pairs)
                            temp_pair = all_collision_pairs{p}; % should be [2*2]: [finger_1, link_1; finger_2, link_2]
                            if ismember([idx_f,idx_p],temp_pair,'rows') && ismember([k_f,k_l],temp_pair,'rows')
                                already_exist = true;
                                break;
                            end   
                        end
                    end

                    if already_exist % if this distance has already been registered before
                        continue;
                    else
                        if ispalm(k_f) % This is palm
                            if idx_p == find(hand.F{idx_f}.idx_real_link,1) % if idx_p is the first active link, skip it
                                continue;
                            end

                            nSamples = min([nCylinderAxis, ceil(link.L/(link.radius*2))]);
                            dist2palm = sym(zeros(1,nSamples));
                            for d = 0:(nSamples-1)
                                xi = x1 + (x2-x1)*d/(nSamples-1);
                                dist2palm(d+1) = dot(palm_normal,xi-palm_point(:));
                            end

                            symvars = symvar(dist2palm);
                            subKeys = symvars(~ismember(symvars,X_key)); % symvars that belong to dict_q but not X_key, need to be substituted
                            if ~isempty(subKeys)
                                subValues = hand.q(ismember(dict_q,subKeys)); % subs them using hand default joint angle values
                                dist2palm = subs(dist2palm, subKeys, subValues.');
                            end
                            c(end+1 : end+numel(dist2palm)) = link.radius - dist2palm;
                        else
                            link_coll = hand.F{k_f}.Link{k_l}; % link to collide with current link
                            if link_coll.is_real
                                y1 = link_coll.symbolic.HT_this(1:3,4); % end points of collided link
                                y2 = link_coll.symbolic.HT_next(1:3,4);

                                nSamples = min([nCylinderAxis, ceil(link.L/(link.radius*2)), ceil(link_coll.L/(link_coll.radius*2))]);
                                dist = distanceBTW2Lines(x1,x2,y1,y2,nSamples); % minimum distance between two links (line segments)

                                symvars = symvar(dist);
                                subKeys = symvars(~ismember(symvars,X_key)); % symvars that belong to dict_q but not X_key, need to be substituted
                                if ~isempty(subKeys)
                                    subValues = hand.q(ismember(dict_q,subKeys)); % subs them using hand default joint angle values
                                    dist = subs(dist, subKeys, subValues.');
                                end
                                c(end+1 : end+numel(dist)) = 2*link_r - dist;
                            end
                        end
                        all_collision_pairs{end+1} = [idx_f,idx_p;k_f,k_l];
                    end
                    
                end
                    
            end
        end
    end
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (this link vs. other links)';
    fprintf('%d\n', c_idx(end));

%     fprintf('* [4/5] Collision avoidance (this link vs. other links): ');
%     all_collision_pairs = {};
%     for i = 1:ncp
%         if ~all(os_info{i})
%             continue; % palm does not move, will not collide with other fingers
%         else
%             [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger and link in the finger
%             finger = hand.F{idx_f};
%             nq_pre = min([idx_l, finger.nlink-1]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link). max(finger.n) = 4
% 
%             for j = 1:nq_pre % Iterate over the current link + all father-links on this finger
%                 link = finger.Link{j}; % link j, The link being checked at this moment
%                 if ~link.is_real
%                     continue;
%                 end
%                 x1 = link.symbolic.HT_this(1:3,4); % end points of current link
%                 x2 = link.symbolic.HT_next(1:3,4);
%                 coll = link.collList; % retrieve the links that collide with the current one
% 
%                 for k = 1:numel(coll) % link k collides with current link
%                     [k_f,k_l] = deal(coll{k}(1),coll{k}(2));
%                     if k_f == 0
%                         % intersection between links and the palm
%                         
%                         % palm_point: point on the inner palm surface
%                         palm_point = hand.P.points_inr(1,:).';
%                         palm_normal = hand.P.contact.symbolic.n;
%                         % only check the link endpoints:
%                         % if the all (real) link endpoints don't intersect
%                         % with palm, then no point on the link does
%                         dist_next = palm_normal.' * (x2 - palm_point);
%                         c(end+1) = hand.hand_radius - dist_next;
% 
%                         continue;
%                     end
%                     if k_f == idx_f % skip link on the same finger
%                         continue;
%                     end
% 
%                     link_coll = hand.F{k_f}.Link{k_l}; % link to collide with current link
%                     if ~link_coll.is_real
%                         continue;
%                     end
%                     
%                     % filter out repetitive collision pairs 
%                     already_exist = false; % if the current pair of collision already exists in the collision pair
%                     if ~isempty(all_collision_pairs)
%                         for p = 1:numel(all_collision_pairs)
%                             temp_pair = all_collision_pairs{p}; % should be [2*2]: [finger_1, link_1; finger_2, link_2]
%                             if ismember([idx_f,idx_l],temp_pair,'rows') && ismember([k_f,k_l],temp_pair,'rows')
%                                 already_exist = true;
%                                 break;
%                             end   
%                         end
%                     end
%                     if already_exist
%                         continue;
%                     else
%                         all_collision_pairs{end+1} = [idx_f,idx_l;k_f,k_l];
%                     end
%                     y1 = link_coll.symbolic.HT_this(1:3,4); % end points of collided link
%                     y2 = link_coll.symbolic.HT_next(1:3,4);
%                     
%                     % Here is the min. distance between two 3d line segments.
%                     % Not min. distance between two 3d lines!
%                     dist = distanceBTW2Lines(x1,x2,y1,y2,5); % minimum distance between two links (line segments) N=5: 5 sampling points
%                     % All virtual father links will result in a 1*1 dist.
%                     %%% Substitute all parameters (qi) that are not in X_key with hand joint values
%                     symvars = symvar(dist);
%                     subKeys = symvars(~ismember(symvars,X_key)); % symvars that belong to dict_q but not X_key, need to be substituted
%                     if ~isempty(subKeys)
%                         subValues = hand.q(ismember(dict_q,subKeys)); % subs them using hand default joint angle values
%                         dist = subs(dist, subKeys, subValues.');
%                     end
%                     try
%                         c(end+1 : end+numel(dist)) = 2*link_r - dist;
%                     catch
%                         disp('I don`t know.');
%                     end
%                 end
%                     
%             end
%         end
%     end
%     c_idx(end+1) = numel(c)-sum(c_idx);
%     c_name{end+1} = 'Collision avoidance (this link vs. other links)';
%     fprintf('%d\n', c_idx(end));

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
            if strcmp(obj_i.type,'sph')
                % the other object is a sphere. Add 1 constraint for the
                % distance
                sph_oc = obj_i.ctr;
                sph_radius = obj_i.radius;
                % add a constraint for each of the n axis points
                nb_axpt_this = size(obj_axpt,2); % obj_axpt is (3 x n_points)
                for j = 1:nb_axpt
                    axis_point = obj_axpt(:,k);
                    dist = norm(sph_oc-axis_point);
                    c(end+1) = sph_radius + obj_r - dist;
                end
                for j=1:length(obj.sphereRadius)
                    this_sph_rad = obj.sphereRadius(j);
                    this_sph_ctr = obj.sphereCenter(j);
                    dist = norm(this_sph_ctr - sph_oc);
                    c(end+1) = this_sph_rad + sph_radius - dist;
                end 
            elseif strcmp(obj_i.type,'cyl')
                cyl_radius = obj_i.radius;
                cyl_axpt = obj_i.axPtArray;
                nb_axpt_cyl = size(cyl_axpt,2);
                nb_axpt_this = size(obj_axpt,2); % obj_axpt is (3 x n_points)
                for k=1:nb_axpt_cyl
                    % add constraint for all points in the cylinder of THIS
                    % object
                    for j=1:nb_axpt_this
                        axis_point_cyl = cyl_axpt(:,k);
                        axis_point_this = obj_axpt(:,j);
                        dist = norm(axis_point_this - axis_point_cyl);
                        c(end+1) = cyl_radius + obj_r - dist;
                    end
                    % add constraints for all additional spheres of THIS
                    % object (collision with points in cylinder object)
                    for j=1:length(obj.sphereRadius)
                        this_sph_rad = obj.sphereRadius(j);
                        this_sph_ctr = obj.sphereCenter(j);
                        dist = norm(this_sph_ctr - axis_point(:,k));
                        c(end+1) = this_sph_rad + cyl_radius - dist;
                    end
                end
 
            elseif strcmp(obj_i.type,'comp')
                comp_radius = obj_i.radius;
                comp_axpt = obj_i.axPtArray;
                nb_axpt_comp = size(cyl_axpt,2);
                nb_axpt_this = size(obj_axpt,2); % obj_axpt is (3 x n_points)
                % collision with the cylindrical part of the OTHER object
                for k=1:nb_axpt_comp
                    % add constraint for all points in the cylinder of THIS
                    % object
                    for j=1:nb_axpt_this
                        axis_point_cyl = comp_axpt(:,k);
                        axis_point_this = obj_axpt(:,j);
                        dist = norm(axis_point_this - axis_point_cyl);
                        c(end+1) = comp_radius + obj_r - dist;
                    end
                    % add constraints for all additional spher
                    for j=1:length(obj.sphereRadius)
                        this_sph_rad = obj.sphereRadius(j);
                        this_sph_ctr = obj.sphereCenter(j);
                        dist = norm(this_sph_ctr - axis_point(:,k));
                        c(end+1) = this_sph_rad + cyl_radius - dist;
                    end
                end
                % collision of the additional spheres of THE OTHER object
                for k=1:length(obj_i.sphereRadius)
                    other_sph_rad = obj_i.sphereRadius(k);
                    other_sph_ctr = obj_i.sphereCenter(k);
                    % loop over cylindrical part of THIS object
                    for j=1:nb_axpt_this
                        axis_point_this = obj_axpt(:,j);
                        dist = norm(axis_point_this - other_sph_ctr);
                        c(end+1) = other_sph_rad + obj_r - dist;
                    end
                    % loop over spheres of THIS object
                    for j=1:length(obj.sphereRadius)
                        this_sph_rad = obj.sphereRadius(j);
                        this_sph_ctr = obj.sphereCenter(j);
                        dist = norm(other_sph_ctr - this_sph_ctr);
                        c(end+1) = other_sph_rad  + this_sph_rad - dist;
                    end
                end
            else
                warning('constraint definition: object type invalid\n');
            end
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