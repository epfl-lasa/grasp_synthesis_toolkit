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
        oc = X_sol(param.idx_oc);       % object center
        quat = X_sol(param.idx_quat);   % quaternion (already normalized)
        q = quaternion(quat(1),quat(2),quat(3),quat(4));
        radius = param.obj_radius; % object radius
        height = param.obj_height;
        
        object = objCylinder(radius,height,q,oc);
    otherwise
    	error('NotImplementedError.');
end