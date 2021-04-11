function cyl = objCylinder(rad, h, quat,t, res)
%objCylinder creates a cylinder object

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
if nargin < 5 % default resolution
    res = 20;
end
if nargin < 4
    t = [0;0;0]; % zero translation
end
if nargin < 3 % default quaternion: no rotation
    quat = quaternion(0,0,0,1);
end
if nargin < 2 % default height
    h = 10;
end
if nargin < 1 % default radius
    rad = 3;
end
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
n = 5;                                    % expression for n pts along axis
c = sym([zeros(2,n);h*(linspace(0,1,n)-0.5)]); % dim (3 x n), z in [-h/2, h/2]
quat_sym = sym('p',[4,1]);                % symbolic expression for quaternion
c_rot = rotate_point(c,quat_sym);    % use rotate_point for sym (not rotatepoint)
c_trans = Htr*[c_rot;ones(1,n)];
c = c_trans(1:3,:);                       % dim (3 x n)

% computation of the projection of the contact point on the axis
mu = sym('mu',[2,1]);
%beta = sym('beta',[2,1]); % this should not be required!
ctr = sym(['x';'y';'z']);


% define the HT from OF to WF
quat_sym_conj = [quat_sym(1);-quat_sym(2);-quat_sym(3);-quat_sym(4)];

% [TODO] clean this part
%x1 = rotate_point([1;0;0],quat_sym_conj);
%x2 = rotate_point([0;1;0],quat_sym_conj);
x3 = rotate_point([0;0;1],quat_sym_conj);
%rotm = [x1;x2;x3];
%HTobj2w = [rotm,t;0,0,0,1]; % transfromation from object frame to world frame

n = x3; % normal expressed in symbolic form
%cp = sym([]);
cp_proj = sym([]);
for i=1:2
    %cp(:,i) = HTobj2w*[rad*(cos(beta(i))+sin(beta(i)));r*(-sin(beta(i))+cos(beta(i)));mu(i)*h;1];
    % contact point projection on OF coord
    cp_proj(:,i) = ctr + mu(i)*h*n;  % contact point projection in WF coord
end
% retrieve points in shape (n1,n2) (for plotting)
X = reshape(p(:,1),n2,n1)';
Y = reshape(p(:,2),n2,n1)';
Z = reshape(p(:,3),n2,n1)';

%% numerical values
% coordinates in the body frame [unused]
struct.X = X;
struct.Y = Y;
struct.Z = Z;

% constant values (used as initial values for optimization)
struct.ctr = center;   
struct.quat = quat;         % quaternion to rotate coordinates OF to WF

%% symbolic expressions (used for constraint description)
struct.ctr_sym = ctr;% center of the object
struct.quat_sym = quat_sym;         % q1,q2,q3,q4 (for optimization)
struct.n = n;  % axis of the cylinder
struct.axpt = c;                    % coordinates of equidistant spheres
%struct.cp =cp;
struct.cp_proj = cp_proj;           % (3 x ncp) projected CP (symbolic)
%% constant fields
struct.radius = rad;
struct.height = h;
struct.type = 'cyl';
struct.res = res;

cyl = struct;
