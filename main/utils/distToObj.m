function dist = distToObj(object, point, evaluation)
% DistToObj returns the radial distance to the axis of a cylindrical object


L = object.height;
if strcmp(evaluation, 'symbolic')
    % the object is the one to be optimized for
    % use symbolic expressions for the points
    x0 = object.sym.c0;
    x1 = object.sym.c1;
elseif strcmp(evaluation, 'fixed')
    % the given object is fixed in space
    % take fixed points for endpoints
    x0 = object.c0;
    x1 = object.c1;
else
    warning('distToObj: Unspecified evaluation method.');
    x0 = zeros(3,1);
    x1 = zeros(3,1);
    L = -1;
end
    
cross_prod = cross(point - x0, x1 - x0);
dist = norm(cross_prod)/L;

end