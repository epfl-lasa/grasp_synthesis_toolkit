% Obtain the symbolic form of nonlinear inequality constraints
function [ceq, ceq_grad, param, ht_ceq, ht_ceq_grad] = compNonLinEqConst(hand, param)
    os_info = param.os.os_info;
    ncp = param.ncp;            % number of contact points
    objRad = param.obj.radius;  % object radius
    type = param.obj.type;
    
    % symbolic expressions from the object
    objCpProj = param.obj.sym.cpProj;% projection on cylinder axis f(x,y,z,quat,mu)
    objCtr = param.obj.sym.ctr;        % object center
    objN = param.obj.sym.n;       % object axis
    
    idxOc = param.idx_oc;
    idxQuat = param.idx_quat;
    idxMu = param.idx_mu;
    
    k = param.k; % number of edges of approximated friction cone
    X_key = param.X_key;
    cstr = param.cstr;
    oc = X_key(idxOc); % object center
    quat = X_key(idxQuat);
    %mu = X_key(idx_mu);
    
    fprintf('\nConstructing Nonl. Equality Constraints: \n');
    if ~strcmp(type, 'comp')
        warning("Nonlinear Inequality Constraint: object type inconsistent");
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Equality Constraint 1: Contact points on object surface
    % dist. between contact points and object ctr equals object radius
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* In-contact constraint: ');
    ceq = sym([]); % to save the symbolic form
    ceq_idx = [];
    ceq_name = {};
    
    for i = 1:ncp
        if ~all(os_info{i}) % This is the palm
            pc = hand.P.contact.symbolic.p;         % palm contact
            cpProj = objCpProj(:,i);                % cp-projection as f(x,y,z,quat,mu)
            del = pc - cpProj;
            ceq(end+1) = (objN.' * del)^2;          % n orthogonal to (cp-cpProj)
            ceq(end+1) = norm(del,2)^2 - objRad^2;  % d(cp, cpProj) = R
        else
            % definition of contact points on fingers
            [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2));
        
            finger = hand.F{idx_f};
            link = finger.Link{idx_l};
            L = link.L;
            rho_sym = ['rho',num2str(idx_f),num2str(idx_l)]; % name of rho associated with this contact

            % cp: contact point on the hand
            cp = hand.F{idx_f}.Link{idx_l}.contact.symbolic.p; 
            % cpProj: projection of the contact point
            cpProj = objCpProj(:,i);  % cp-projection as f(x,y,z,quat,mu)
            del_cp = cp - cpProj;                             
            c1 = (objN.' * del_cp)^2; % projection = 0 (take squared constraint!)
            c1 = subs(c1, {'L',rho_sym},{L,link.radius});
            ceq(end+1) = c1;          % add projecton constraint
                
            c2 = norm(del_cp, 2)^2-objRad^2; % distance between cp and cpProj is R
            c2 = subs(c2, {'L',rho_sym},{L,link.radius});
            ceq(end+1) = c2;          % add length constraint
        end
         % notice that cp_dist is squared; so: cp_dist == r*r;
    end
    ceq_idx(end+1) = numel(ceq);
    ceq_name{end+1} = 'In-contact constraint';
    fprintf('%d\n', ceq_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Equality Constraint 2: Force closure
    % Coefficients times wrench cone edges, sum up to 0(6,1)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if cstr.fc
        fprintf('* Force closure: ');
        W = sym(zeros(6,ncp*k)); % A list to save all wrenches
        c = sym('c%d%d',[ncp,k]); % coefficients of friction cone edges in solving the linear programming problem
        c = reshape(c.',[],1);
        for i = 1:ncp
            if ~all(os_info{i}) % palm
                FC_i = hand.P.contact.symbolic.FC;
                TC_i = hand.P.contact.symbolic.TC;
            else
                [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2));
                finger = hand.F{idx_f};
                link = finger.Link{idx_l};

                rho_sym = ['rho',num2str(idx_f),num2str(idx_l)];

                FC_i = link.contact.symbolic.FC; % (3,k)
                FC_i = subs(FC_i, {'L','r',rho_sym}, {link.L,objRad,link.radius});

                TC_i = link.contact.symbolic.TC; % (3,k)
                TC_i = subs(TC_i, {'L','r',rho_sym}, {link.L,objRad,link.radius});
            end
            W_i = cat(1,FC_i,TC_i); % (6,k) construct wrench vector
            W(:,(i-1)*k+1:i*k) = W_i; % (6,ncp*k) stack wrenches from this contact point to the entire wrench
        end
        ceq(end+1:end+6) = W*c;
    end
    ceq_idx(end+1) = numel(ceq)-sum(ceq_idx);
    ceq_name{end+1} = 'Force closure';
    fprintf('%d\n', ceq_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Equality constraint 3: normalized quaternion
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* normalize quaternion :')
    ceq(end+1)= norm(quat,2)^2 - 1;
    ceq_idx(end+1) = numel(ceq) - sum(ceq_idx);
    ceq_name{end+1} = 'Quaternion normalization';
    fprintf('%d\n', ceq_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save symbolic form of nonlinear constraints
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    param.ceq_idx = ceq_idx;
    param.ceq_name = ceq_name;
    
    matlabFunction(ceq,'File','../database/symbolic_functions/nonl_ceq','Vars',X_key,'Optimize',false);
    
    ceq_grad = transpose(jacobian(ceq, X_key)); % size (lens_var, lens_fun)
    
    matlabFunction(ceq_grad,'File','../database/symbolic_functions/nonl_ceq_grad','Vars',X_key,'Optimize',false);
    
    fprintf('  Total num. of nonl. equality constraints: %d\n', numel(ceq));
    
    if nargout > 3
        ht_ceq = matlabFunction(ceq,'Vars',X_key);
        ht_ceq_grad = matlabFunction(ceq_grad,'Vars',X_key);
    end
end