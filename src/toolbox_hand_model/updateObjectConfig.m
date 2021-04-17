function object = updateObjectConfig(object, soln)

X_sol = soln.X_sol;
param = soln.param;

switch object.type
    case 'sph' % sphere
        oc = X_sol(param.idx_oc); % object center
        To = trvec2tform(oc(:).');
        radius = param.object_radius; % object radius
        
        object = mySGsphere(To,radius,object.clr);
    case 'cyl' % cylinder
        cylParam.transl = X_sol(param.idx_oc);     % object center
        quat = X_sol(param.idx_quat);         % quaternion (already normalized)
        cylParam.quat = quaternion(quat(1),quat(2),quat(3),quat(4));
        cylParam.radius = param.obj.radius;   % object radius
        cylParam.height = param.obj.height;   % object height
        cylParam.res = 20;                    % resolution of the cylinder
        
        % projection of the contact points on central axis (check validity)
        mu = X_sol(param.idx_mu);
        
        object = cylinderObj(cylParam,mu);
    otherwise
    	error('NotImplementedError.');
end