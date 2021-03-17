function cyl = objCylinder(rad, h, res, quat)
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
if nargin < 4
    quat = quaternion(0,0,0,1);
end
if nargin < 3
    res = 20;
end
if nargin < 2
    h = 10;
end
if nargin < 1
    rad = 3;
end

% generate a cylinder of the given size
[X,Y,Z] = cylinder(rad,res);
X = X;
Y = Y;
Z = h*(Z-0.5);

[n1,n2] = size(X);

X = reshape(X',n1*n2,1);
Y = reshape(Y',n1*n2,1);
Z = reshape(Z',n1*n2,1);

p = rotatepoint(quat,[X,Y,Z]);

X = reshape(p(:,1),n2,n1)';
Y = reshape(p(:,2),n2,n1)';
Z = reshape(p(:,3),n2,n1)';

% coordinates in the body frame
struct.X = X;
struct.Y = Y;
struct.Z = Z;

struct.center = [0;0;0]; % center always at the origin
struct.n = rotatepoint(quat,[0,0,1]); % axis of the cylinder
struct.res = res;
struct.type = 'cyl';
struct.radius = rad;
struct.height = h;

cyl = struct;
