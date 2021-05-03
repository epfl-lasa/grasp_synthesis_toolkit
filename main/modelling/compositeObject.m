function [object] = compositeObject(params)
%%%
if ~isfield(params,'res') % default resolution
    params.res = 20;
end
if ~isfield(params,'transl')
    params.transl = [0;0;0]; % zero translation
end
if ~isfield(params,'quat') % default quaternion: no rotation
    params.quat = quaternion(0,0,0,1);
end
if ~isfield(params,'height') % default height
    params.height = 10;
end
if ~isfield(params,'radius') % default radius
    params.rad = 3;
end
height = params.height;
radius = params.radius;
quat = params.quat;
transl = params.transl;
res = params.res;

sphereCenter = params.sphereCenter; % (N x 3) array
sphereRadius = params.sphereRadius; % (N x 1) array

%%% define symbolic variables
symCtr = sym(['x';'y';'z']);
symQuat = sym('q',[4,1]);
symNormal = rotate_point([0;0;1],symQuat);
symMu = sym('mu',[2,1]);
symCpProj = sym([]);
for i=1:2
    % contact point projection on OF coord
    symCpProj(:,i) = symCtr + symMu(i)*height*symNormal;  % contact point projection in WF coord
end

%%% define center of the object
Htr = eye(4);               % HT: transform coord. from OF to WF
Htr(1:3,4) = transl;             
ctr = Htr*[0;0;0;1];     % center only translated (rotation invariant)
ctr = ctr(1:3);

% computation of points on the cylinder surface
[X,Y,Z] = cylinder(radius,res); % size (2, res) for X,Y and Z
Z = height*(Z-0.5); % recenter and scale to length
[n1,n2] = size(X);

%%% pointcloud to define the cylinder
X = reshape(X',n1*n2,1); % size (2*res, 1)
Y = reshape(Y',n1*n2,1);
Z = reshape(Z',n1*n2,1);

p = rotatepoint(quat,[X,Y,Z]); % points are (2*res,3)
p = [p,ones(n1*n2,1)]; % (2*res,4)
p_transl_t = Htr*p.'; % size (4, 2*res)  -> transposed
p = p_transl_t(1:3,:).'; % translated and rotated points

X = reshape(p(:,1),n2,n1)';
Y = reshape(p(:,2),n2,n1)';
Z = reshape(p(:,3),n2,n1)';

%%% array of points along the axis (symbolic expression)
dmin = radius*cos(asin(0.7));
nPoints = ceil(height/dmin);                       % expression for n pts along axis
pointDist = height/nPoints;
distArray = pointDist/2 + [0:nPoints-1]*pointDist - height/2;
symAxPtArray = sym([zeros(2,nPoints);distArray]); % dim (3 x n), z in [-h/2, h/2]
symQuat = sym('p',[4,1]);                % symbolic expression for quaternion
symAxPtArrayRot = rotate_point(symAxPtArray,symQuat);    % use rotate_point for sym (not rotatepoint)
symHT = sym(eye(4));
symHT(1:3,4) = symCtr;
symAxPtArrayHt = symHT*[symAxPtArrayRot;ones(1,nPoints)];
symAxPtArray = symAxPtArrayHt(1:3,:);                  % dim (3 x n)


axPtArray = [zeros(nPoints,2),distArray.']; % (N x 3)
axPtArray = ctr + rotatepoint(quat,axPtArray).';  % (3 x N)

%%% store values of spheres to add
%   (symbolic expressions for sphere centers)
symSphereCenter = sym([]);
for i=1:length(sphereRadius)
    sphereCtrWF = symHT*[sphereCenter(i,:).';1];
    symSphereCenter(:,i) = sphereCtrWF(1:3);
end
%   (numerical expressions for sphere centers)
sphereCenter = transl + rotatepoint(quat,sphereCenter).'; % transpose to (3 x N)
  
%%% store numerical variables
object.xArray = X;
object.yArray = Y;
object.zArray = Z;

object.ctr = ctr;
object.quat = quat;
object.n = rotatepoint(quat,[0,0,1]).';
object.height = height;
object.radius = radius;
object.type = 'cyl';
object.res = res;
object.axPtArray = axPtArray;
object.sphereCenter = sphereCenter;
object.sphereRadius = sphereRadius;

%%% store symbolic variables
object.sym.center = symCtr;
object.sym.quat = symQuat;
object.sym.n = symNormal;
object.sym.mu = symMu;
object.sym.cp_proj = symCpProj;
object.sym.axPointArray = symAxPtArray;
object.sym.sphereCenter = symSphereCenter;

end