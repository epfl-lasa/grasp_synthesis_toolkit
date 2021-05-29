% Obtain the symbolic form of nonlinear inequality constraints
function [c, c_grad, param, ht_c, ht_c_grad] = sphNonLinIneqConst(hand, param)

    os_info = param.os.os_info;
    
    obj_r = param.obj.radius; % object radius
    link_r = hand.hand_radius;
    
    ncp = param.ncp; % number of contacts
    X_key = param.X_key;
    oc = param.obj.sym.ctr; % object center
    dict_q = param.dict_q; % dictionary of symbolic variables: q11,q12,...
    nCylinderAxis = 5;
    palm_point = mean(hand.P.points_inr,1).';     % point on inner palm surface [3 x 1] array
    palm_normal = hand.P.contact.symbolic.n(:);   % palm surface normal
    
    % build an array containing active finger indices
    f_actv = zeros(1,ncp); 
    for i = 1:ncp
        f_actv(i) = os_info{i}(1);
    end
    % if the palm is in contact, remove the included fingers
    f_actv = f_actv(f_actv ~= 0);
    f_actv = min(f_actv):max(f_actv); % 
    
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
    fprintf('* Collision avoidance (object vs. links): ');
    for i = 1:ncp
        if ~all(os_info{i})
            % collision with palm is guaranteed in 'Collision avoidance (object vs. palm)' 
            continue;
        else
            [idx_f, idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger and link
            finger = hand.F{idx_f};
            if (idx_l > finger.n)
                warning("Nonlinear constraint definition: os contains fingertip");
            end
            nq_pre = min([idx_l, finger.n]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link)
            for j = 1:nq_pre % Iterate over all links, including link in contact ('idx_l')
                link = finger.Link{j};
                if ~link.is_real
                    continue;
                end
                %link_dist = link.symbolic.link_dist;
                x0 = link.symbolic.HT_this(1:3,4);
                x1 = link.symbolic.HT_next(1:3,4);
                link_dist = norm(cross(x1-x0,oc-x0))/link.L;
                c_ij = -(link_dist-link_r-obj_r); % symvar: [q1...q4, x, y, z]
                c(end+1) = c_ij; % c<=0; distance from link to should larger than radius of finger digit cylinder plus radius of object
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
    fprintf('* Collision avoidance (object vs. included fingers): ');
    % included finger: finger that between two active fingers, e.g. the middle finger is included if using index finger and ring finger for grasping

    incl_fingers = min(f_actv)+1:max(f_actv)-1; % indices of in-between fingers
    
    if ~isempty(incl_fingers)
        for f = min(f_actv)+1:max(f_actv)-1
            finger = hand.F{f}; % same as collision avoidance: father-links vs. object
            for j = 1:finger.n-1
                link = finger.Link{j};
                if ~link.is_real
                    continue;
                end
                link_dist = link.symbolic.link_dist; % this is the distance from link central axis to an external 3d point [x,y,z]
                c_ij = -(link_dist-link_r-obj_r); % symvar: [q1...q4, x, y, z]
                c_ij = subs(c_ij, finger.q_sym, finger.q.');
                c(end+1) = c_ij;
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
    fprintf('* Collision avoidance (object vs. palm): ');
   
    % compute the distance of the sphere to the palm using a projection on
    % the normal vector of the palm
   
    dist = dot(palm_normal,oc-palm_point); % project the object on the normal
    c(end+1) = obj_r - dist;
    
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
                
                % construct the constaints only w.r.t finger links
                % that are active
                if ((k_f < min(f_actv)) || (k_f > min(f_actv)))
                    continue;
                end
                
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
%                     if k_f ==0
%                         % palm_point: point on the palm inner surface
%                         palm_point = hand.P.points_inr(1,:).';
%                         palm_normal = hand.P.contact.symbolic.n;
%                         % only check the link endpoints (palm is flat)
%                         % (if constraint is satisfied at all endpoints,
%                         % then there cannot be an intersection with any
%                         % finger and the palm)
%                         dist_next = palm_normal.' * (x2 - palm_point);
%                         c(end+1) = link_r - dist_next;
%                         % [TODO] check if inner surface is at z=-5
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
        fprintf('* Collision avoidance (target object and object in grasp): ');
        nobj = numel(grasped_objects);
        for i = 1:nobj
            obj_i = grasped_objects{i};
            if strcmp(obj_i.type,'sph')
                % the other object is a sphere. Add 1 constraint for the
                % distance
                oc_i = obj_i.ctr;
                radius_i = obj_i.radius;
                c(end+1) = (obj_r+radius_i) - norm(oc(:)-oc_i(:)); 
                % distance between centers of target object and the grasped
                % sphere is larger than the sum of their radius
            elseif strcmp(obj_i.type,'cyl')
                % the grasped object is a sphere. Add distance constraint
                % for all of the nAxis points
                nb_axpt = size(obj_i.axPtArray,2);
                for j=1:nb_axpt
                    axis_point = obj_i.axPtArray(:,j);
                    radius_cyl = obj_i.radius;
                    c(end+1) = (obj_r+radius_cyl) - norm(oc(:) - axis_point);
                end
            elseif strcmp(obj_i.type,'comp')
                % the grasped object is a composite object.
                % Add constraints for the cylinder plus for additional
                % spheres
                nb_axpt = size(obj_i.axPtArray,2);
                for j=1:nb_axpt
                    axis_point = obj_i.axPtArray(:,j);
                    radius_cyl = obj_i.radius;
                    c(end+1) = (obj_r+radius_cyl) - norm(oc(:) - axis_point);
                end
                n_spheres = length(obj_i.sphereRadius);
                for j=1:n_spheres
                    sphere_center = obj_i.sphereCenter(j);
                    radius_sph = obj_i.sphereRadius(j);
                    c(end+1)=(obj_r + radius_sph) - norm(oc(:) - sphere_center);
                end
            else
                warning('Constraint definition: object type not valid\n');
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