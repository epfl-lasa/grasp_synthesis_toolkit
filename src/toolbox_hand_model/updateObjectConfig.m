function object = updateObjectConfig(object, soln)

X_sol = soln.X_sol;
param = soln.param;

switch object.type
    case 'sph' % sphere
        oc = X_sol(param.idx_oc); % object center
        transl = [oc(1);oc(2);oc(3)];
        radius = param.obj.radius; % object radius
        object = sphereObject(transl,radius,object.clr);
    case 'cyl' % cylinder
        cylParam.transl = X_sol(param.idx_oc);     % object center
        quat = X_sol(param.idx_quat);         % quaternion (already normalized)
        cylParam.quat = quaternion(quat(1),quat(2),quat(3),quat(4));
        cylParam.radius = param.obj.radius;   % object radius
        cylParam.height = param.obj.height;   % object height
        cylParam.res = 20;                    % resolution of the cylinder
        
        % projection of the contact points on central axis (check validity)
        mu = X_sol(param.idx_mu);
        
        object = cylinderObject(cylParam,mu);
    case 'comp'
        param.transl = X_sol(param.idx_oc);
        param.height = param.obj.height;
        param.radius = param.obj.radius;
        quat = X_sol(param.idx_quat);
        param.quat = quaternion(quat(1),quat(2),quat(3),quat(4));
        param.res = 20;
        param.sphereCenter = param.obj.relSphereCenter;
        param.sphereRadius = param.obj.sphereRadius;
        object = compObject(param);
        
        
    otherwise
    	error('NotImplementedError.');
end