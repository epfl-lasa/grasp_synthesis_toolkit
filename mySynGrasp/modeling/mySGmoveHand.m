%    mySGmoveHand - Move a hand
%
%    The function move the finger joint imposing a displacement q. Returns
%    the hand in the new configuration.
%
%    Usage: hand = mySGmoveHand(hand,q)
%
%    Arguments:
%    hand = the hand structure to move
%    q = the joint variables of the new hand configuration
%    plot = this argument allows the user to plot the resulting movement of
%    the hand with the following semantic:
%       'initfinal' = the initial and final positions of the hand are
%                     plotted
%       'final' = only the final position of the hand is plotted
%
%    Returns:
%    hand = the hand in the new configuration
%
%    See also: mySGmakehand, mySGmakeFinger
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


function newHand = mySGmoveHand(hand,q)

    if nargin < 2
        q = hand.q; % if no extra q is given, use the current q to update the hand
    end

	% Input arguments consistency check
    if(~mySGisHand(hand))
        error 'hand argument is not a valid hand-structure' 
    end
    if(isstruct(q))
       error 'q argument should be an array such that size(q) == size(hand.q)' 
    end
    
    if(size(q,1) == 1)
        q = q';
    end
    if(size(q,1) ~= size(hand.q,1))
        error('mySG: Invalid dimensions of the input arguments.');
    end
    
    newHand = hand;

    if hand.type == "AllegroHandLeft"
        theta_fingers_allegro = [0,pi/2,0,0]';
        theta_thumb_left_allegro = [0,pi/2,-pi/2,0]';
        matrix_allegro = [theta_fingers_allegro,...
            theta_fingers_allegro,...
            theta_fingers_allegro,...
            theta_thumb_left_allegro];
    end
    if hand.type == "AllegroHandRight" 
        theta_fingers_allegro = [0,pi/2,0,0]';
        theta_thumb_right_allegro = [0,pi/2,pi/2,0]';
        matrix_allegro = [theta_fingers_allegro,...
            theta_fingers_allegro,...
            theta_fingers_allegro,...
            theta_thumb_right_allegro];
    end
        
    for i = 1:length(q)
        if hand.qactv(i) == 0 % the joint is inactive
            q(i) = hand.q(i);
        end
    end
    
	k = 1;
	for i = 1:hand.n
        F_old = hand.F{i};
        DHpars = F_old.DHpars;
        n = F_old.n;

        %{
        if hand.type=="AllegroHandLeft" || hand.type=="AllegroHandRight" 
            DHpars(:,3) = q(k:k+n-1)+matrix_allegro(:,i); 
            % Here we add the DH pars of the hand model as they are defined
            % in mySGallegroLeft and in mySGallegroRight. In this way, the hand
            % position restarts always in the initial configuration defined
            % in those files. This is different from other hands in
            % SynGrasp because the Allegro Hand has non-zero theta
            % parameters in the DH tables.
        else
            DHpars(:,3) = q(k:k+n-1);
        end
        
        newHand.F{i} = mySGmakeFinger(DHpars,F_old.base,q(k:k+n-1));
        %}

        newHand.F{i} = moveFinger(hand.F{i}, q(k:k+n-1));
        k = k + n;
	end

	newHand.q = q;
	newHand.ftips = mySGfingertips(newHand);

	tmp_cp = hand.cp;
	newHand.cp = [];

	for i = 1:size(tmp_cp,2)    
        newHand = mySGaddContact(newHand,tmp_cp(7,i),tmp_cp(4,i),tmp_cp(5,i),tmp_cp(6,i));
	end
    
	newHand = mySGjoints(newHand);
