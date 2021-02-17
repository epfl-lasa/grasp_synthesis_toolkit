function mySGplotHand(hand, transp)

    if nargin < 2
        transp = 0.9; % transparancy of plotting [0 for no color]
    end
    if nargin < 1
        hand = mySGparadigmatic;
    end

    if(~mySGisHand(hand))
        error 'hand argument is not a valid hand-structure' 
    end
    
    nf = hand.n; % number of fingers
    
    for j = 1:nf % for each finger. Order: Index, Thumb, Middle, Ring, Little
        F = hand.F{j};
        
        % plot the finger base
        plot3(F.base(1,4),F.base(2,4),F.base(3,4),'ro'); % base position in World
        if(j == 1)
            hold on;
        end

        % plot the joints and the end tip
        referenceJoint = F.base;
        for i = 2:F.n+1
            localTransf = mySGDHMatrix(F.DHpars(i-1,:));
            
            refOld = referenceJoint(1:3,4);
            referenceJoint = referenceJoint*localTransf;
            
            p1 = [refOld(1),refOld(2),refOld(3)]';
            p2 = [referenceJoint(1,4),referenceJoint(2,4),referenceJoint(3,4)]';
            
            % plot link (finger digit)
            if hand.type == "AllegroHandLeft" || hand.type == "AllegroHandRight" 
               if i < F.n+1
                   mySGplotLink_allegro(p1,p2,[28/2 28/2],transp,[0.30 0.30 0.30]);
               else           
                   mySGplotLink_allegro(p1,p2,[28/2 28/2],transp,[0.9 0.9 0.9]);
               end
            else
                mySGplotLink(p1,p2,5,transp);
            end
            
            % plot the joint location
            if i < F.n+1
               h = plot3(referenceJoint(1,4),referenceJoint(2,4),referenceJoint(3,4),'ro');
               set(h,'MarkerSize',8);
               set(h,'LineWidth',3);
            end
        end
    end
    
    % plot hand palm
    if hand.type == "AllegroHandLeft" || hand.type == "AllegroHandRight" 
        mySGplotPalm_allegro(hand);
    else
        mySGplotPalm(hand);
    end
    
    axis 'equal';
    grid on;
%     view([-150, 30]);

    xlabel('X','FontSize',12);
    ylabel('Y','FontSize',12);
    zlabel('Z','FontSize',12);
    title(hand.type);
    
    hold on;
    
end