%    mySGunderActuated1- Defines the kinematic structure of an underactuated
%    three fingered robotic hand
%
%    Usage: hand = mySGunderActuated1
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D.
%     Prattichizzo,
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


function newHand = mySGunderActuated1(T)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    1 -   H A N D    P A R A M E T E R S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 1
  T = eye(4);
end

%%% Pre-allocation
DHpars{3} = [];
base{3} = [];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Thumb
DHpars{1}=[
   0 38 0 0;
   0 38 0 0;
]; 
base{1} = [
    1 0 0 48;
    0 0 1 0;
    0 -1 0 0;
    0 0 0 1;
];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Index
DHpars{2}=[   
   0 38 0 0;
   0 38 0 0;
];
base{2} = [
    0 0 -1 0;
    1 0 0 52;
    0 -1 0 0;
    0 0 0 1;
];
  

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Middle
DHpars{3}=[   
   0 38 0 0;
   0 38 0 0;
];
base{3} = [    
    -1 0 0 -48;
    0 0 -1 0;
    0 -1 0 0;
    0 0 0 1;
];


%%% Pre-allocation
F{3} = [];
 
for i = 1:length(DHpars)
    % number of joints for each finger
    joints = size(DHpars{i},1);
    % initialize joint variables
    q = zeros(joints,1);
    % make the finger
    F{i} = mySGmakeFinger(DHpars{i},T*base{i},q);
end


newHand = mySGmakeHand(F,T);
newHand.type = 'Modular';