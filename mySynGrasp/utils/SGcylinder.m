%    SGcube - function that calculates main data of a cylinder needed by 
%    grasping-related applications.
%                                                      3
%    Usage: struct = SGcylinder(Htr,h,rad,res)
%
%    Arguments:
%    Htr = Homogeneous transformation
%    h = height of the cylinder
%    rad = radius
%    res = number of points around circumference
%
%    Returns:
%    struct = a structure containing the cylinder characteristics
%
%    See also: SGcloseHand, SGgraspPlanner, SGmakeObject
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D. Prattichizzo,
%  All rights reserved.
%
%  Redistribution and use with or without
%  modification, are permitted provided that the following conditions are met:
%      * Redistributions of source code must retain the above copyright
%        notice, this list of conditions and the following disclaimer.
%      * Redistributions in binary form must reproduce the above copyright
%        notice, this list of conditions and the following disclaimer in the
%        documentation and/or other materials provided with the distribution.
%      * Neither the name of the <organization> nor the
%        names of its contributors may be used to endorse or promote products
%        derived from this software without specific prior written permission.
%
%  THIS SOFTWARE IS PROVIDED BY M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo, ``AS IS'' AND ANY
%  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
%  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
%  DISCLAIMED. IN NO EVENT SHALL M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
%  EXEMPLARY, OR CONSEQUENTIAL DAMAGES
%  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function struct = SGcylinder(Htr,h,rad,res)
if nargin == 0
    R = SGroty(-pi/6);
    H = [R,zeros(3,1);zeros(1,3),1];
    H(1:3,4) = [-10,60,-40];
    Htr=H;
    h=80;
    rad=40;
    res=50;
    [X,Y,Z] = cylinder(rad);
end
if nargin == 1
    h=80;
    rad=40;
    res=50;
    [X,Y,Z] = cylinder(rad);
end   
if nargin == 2
    rad=40;
    res=50;
    [X,Y,Z] = cylinder(rad,res);        
end
if nargin == 3
    res = 50; 
    [X,Y,Z] = cylinder(rad);
end   
if nargin == 4
    [X,Y,Z] = cylinder(rad,res);        
end

c1 = Htr*SGtransl([0,0,-h/2]);
c2 = Htr*SGtransl([0,0,h/2]);
struct.type = 'cyl';
struct.axis = c1(1:3,4)-c2(1:3,4)/norm(c1(1:3,4)-c2(1:3,4)); % axis direction
struct.center = Htr(1:3,4);
Z = h*Z; %height accordingly to requested

struct.p = zeros(3,size(X,2),size(X,1));
for k=1:size(X,1)
    for j=1:size(X,2)
        p = [X(k,j);Y(k,j);Z(k,j);1];
        v = Htr*SGtransl([0,0,-h/2])*p;
        struct.p(:,j,k) = v(1:3);
    end
end

for i=1:size(X,1)
    for j=1:size(X,2)
        struct.X(i,j) = struct.p(1,j,i);
        struct.Y(i,j) = struct.p(2,j,i);
        struct.Z(i,j) = struct.p(3,j,i);
    end
end

struct.radius = rad;
struct.Htr = Htr;
struct.h = h;
struct.res = res;

end