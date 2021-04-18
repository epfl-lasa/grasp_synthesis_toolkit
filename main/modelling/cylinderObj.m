function Cylinder = cylinderObj(cylParam,solMu)
% objCylinder creates a cylinder object

% Arguments:
% rad = radius of the cylinder
% h = height of the cylinder (along z-axis in object frame)
% res = resolution (points on the circumference)
% quat = quaternion to rotate the cylinder about
%
% Output: 
% cylinder (structure with fields
%   - type
%   - res (resolution)
%   - center
%   - quat (quaternion for coord.transf. object to world frame
%   - vertices (vertices of the approxiated cylinder)
% add default parameters

%% set default parameters
if ~isfield(cylParam,'res') % default resolution
    cylParam.res = 20;
end
if ~isfield(cylParam,'transl')
    cylParam.t = [0;0;0]; % zero translation
end
if ~isfield(cylParam,'quat') % default quaternion: no rotation
    cylParam.quat = quaternion(0,0,0,1);
end
if ~isfield(cylParam,'height') % default height
    cylParam.height = 10;
end
if ~isfield(cylParam,'radius') % default radius
    cylParam.rad = 3;
end
h = cylParam.height;
rad = cylParam.radius;
quat = cylParam.quat;
t = cylParam.transl;
res = cylParam.res;
%%
% computation of the center
Htr = eye(4);               % HT: transform coord. from OF to WF
Htr(1:3,4) = t;             
center = Htr*[0;0;0;1];     % center only translated (rotation invariant)
center = center(1:3);

% computation of points on the cylinder surface
[X,Y,Z] = cylinder(rad,res); % size (2, res) for X,Y and Z
Z = h*(Z-0.5); % recenter and scale to length
[n1,n2] = size(X);

X = reshape(X',n1*n2,1); % size (2*res, 1)
Y = reshape(Y',n1*n2,1);
Z = reshape(Z',n1*n2,1);


p = rotatepoint(quat,[X,Y,Z]); % points are (2*res,3)
p = [p,ones(n1*n2,1)]; % (2*res,4)
p_transl_t = Htr*p.'; % size (4, 2*res)  -> transposed
p = p_transl_t(1:3,:).'; % translated and rotated points

% equidistant points along the axis (symbolic expression)
dmin = rad*cos(asin(0.5));
nPoints = ceil(h/dmin);                       % expression for n pts along axis
pointDist = h/nPoints;
distArray = pointDist/2 + [0:nPoints-1]*pointDist - h/2;
symAxPtArray = sym([zeros(2,nPoints);distArray]); % dim (3 x n), z in [-h/2, h/2]
symQuat = sym('p',[4,1]);                % symbolic expression for quaternion
symAxPtArrayRot = rotate_point(symAxPtArray,symQuat);    % use rotate_point for sym (not rotatepoint)
symCtr = sym(['x';'y';'z']);
symHT = sym(eye(4));
symHT(1:3,4) = symCtr;
symAxPtArrayHt = symHT*[symAxPtArrayRot;ones(1,nPoints)];
% [old] symAxPtArrayHt = Htr*[symAxPtArrayRot;ones(1,nPoints)];
symAxPtArray = symAxPtArrayHt(1:3,:);                  % dim (3 x n)

% equidistant points (numerical)
axPtArray = [zeros(nPoints,2),distArray.']; % (N x 3)
axPtArray = center + rotatepoint(quat,axPtArray).';  % (3 x N)

% computation of the projection of the contact point on the axis
symMu = sym('mu',[2,1]);
symNormal = rotate_point([0;0;1],symQuat); % normal expressed in symbolic form

cp_proj = sym([]);

for i=1:2
    % contact point projection on OF coord
    cp_proj(:,i) = symCtr + symMu(i)*h*symNormal;  % contact point projection in WF coord
end
% retrieve points in shape (n1,n2) (for plotting)
X = reshape(p(:,1),n2,n1)';
Y = reshape(p(:,2),n2,n1)';
Z = reshape(p(:,3),n2,n1)';

%% constant fields 
% (used as initial values for optimization)

% [2 x res] arrays containing points to plot the surface
Cylinder.xArray = X;
Cylinder.yArray = Y;
Cylinder.zArray = Z;

Cylinder.ctr = center;   
Cylinder.quat = quat;         % quaternion to rotate coordinates OF to WF
Cylinder.n = rotatepoint(quat,[0,0,1]).';
Cylinder.radius = rad;
Cylinder.height = h;
Cylinder.type = 'cyl';
Cylinder.res = res;
Cylinder.axPtArray = axPtArray;

if nargin >1 
    % if the solution is already calculated,
    % store the projected contact points to display
    Cylinder.cp = [ center + rotatepoint(quat, [0,0,h*solMu(1)]).',...
                    center + rotatepoint(quat, [0,0,h*solMu(2)]).'];
    Cylinder.mu = solMu;
end
%% symbolic expressions (used for constraint description)
Cylinder.sym.ctr = symCtr;% center of the object
Cylinder.sym.quat = symQuat;         % q1,q2,q3,q4 (for optimization)
Cylinder.sym.n = symNormal;  % axis of the cylinder
Cylinder.sym.axisPtArray = symAxPtArray;        % (3 x n) coordinates of equidistant spheres
Cylinder.sym.cpProj = cp_proj;   % (3 x ncp) projected CP (symbolic)
