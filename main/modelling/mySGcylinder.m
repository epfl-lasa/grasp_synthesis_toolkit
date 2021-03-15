% mySGcylinder creates a cylinder object

% Arguments:
% H = homogenous transform from object frame to world frame 
% [TODO] check if this is necessary..
% rad = radius of the cylinder
% h = height of the cylinder (along z-axis in object frame)
%
% Output: 
% cylinder (structure with fields
%   - type
%   - res (resolution)
%   - center
%   - quat (quaternion for coord.transf. object to world frame
%   - vertices (vertices of the approxiated cylinder)
%
% adapted from the SynGrasp Toolbox (Synergy Grasping Toolbox):
%      Copyright (c) 2012 M. Malvezzi, G. Gioioso, G. Salvietti, D.
%        Prattichizzo, A. Bicchi
% 
%        This file is part of SynGrasp (Synergy Grasping Toolbox).
% 
%      All rights reserved.
% 
%      Redistribution and use with or without
%      modification, are permitted provided that the following conditions are met:
%          * Redistributions of source code must retain the above copyright
%            notice, this list of conditions and the following disclaimer.
%          * Redistributions in binary form must reproduce the above copyright
%            notice, this list of conditions and the following disclaimer in the
%            documentation and/or other materials provided with the distribution.
%          * Neither the name of the <organization> nor the
%            names of its contributors may be used to endorse or promote products
%            derived from this software without specific prior written permission.
% 
%      THIS SOFTWARE IS PROVIDED BY M. Malvezzi, G. Gioioso, G. Salvietti, D.
%      Prattichizzo, ``AS IS'' AND ANY
%      EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
%      WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
%      DISCLAIMED. IN NO EVENT SHALL M. Malvezzi, G. Gioioso, G. Salvietti, D.
%      Prattichizzo BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
%      EXEMPLARY, OR CONSEQUENTIAL DAMAGES
%      (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%      LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%      ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%      SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function cyl = mySGcylinder(rad, h, res, Htr)
% add default parameters
if nargin < 4
    % no holonomic tranform given, define a random normal vector
    n = randn(3,1);
    n = n./norm(n);
    
    theta = pi*rand(1);
    
    % quaternion to rotate object frame to world frame
    quat = quaternion([cos(theta/2),sin(theta/2)*n']);
    
    % TODO reconstruct holonomic transform
    H = eye(4);
else 
    % TODO extract holonimic transform
end
if nargin < 3
    res = 20;
end
if nargin < 1
    rad = 20;
    h = 30;
end

[X,Y,Z] = cylinder(rad,res);

X = reshape(X',2*(res+1),1);
Y = reshape(Y',2*(res+1),1);
% body-frame is centered in the middle of the object
Z = reshape(Z',2*(res+1),1) - 0.5;
% dilate the cylinder to its correct height
Z = Z*h;
vertices = [X,Y,Z]; % size N x 3

center = H(1:3,4);

% rotate object frame to world frame
vertices = rotateframe(quat, vertices);

X = vertices(:,1);
Y = vertices(:,2);
Z = vertices(:,3);

X = reshape(X,2,res+1);
Y = reshape(Y,2,res+1);
Z = reshape(Z,2,res+1);

struct.type = 'cyl';
struct.res = res;

struct.ctr = center;
struct.X = X;
struct.Y = Y;
struct.Z = Z;



cyl = struct;
