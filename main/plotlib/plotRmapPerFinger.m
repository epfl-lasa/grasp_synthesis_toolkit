function plotRmapPerFinger(hand, hand_map, fingersToPlot)
% plot the reachability map
% map_hand: the reachability map
% finger_idx: cell, {}, 1*nf, indices, map_idx{i} indicates the indices of
% links of finger i. if NaN, plot all links.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 3
    fingersToPlot = ones(1,5); % default: plot all fingers
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nf = sum(fingersToPlot); % number of fingers (to plot)

% generate different colors for each link
% use the same 4 colors for all fingers
cmap = distinguishable_colors(4);


for f = 1:nf
    % skip if the finger should not be plotted
    if ~fingersToPlot(f)
        continue;
    end
    cidx = 1; % cidx = 4+4; % color index. counter of link to plot, used to select colors
    
    %%% plot hand model
    figure
    mySGplotHand(hand); % plot robotic hand at its initial position
    hold on;

    map_finger = hand_map{f};
    
    
    if isa(map_finger,'struct') % palm is a struct. For fingers, it is cell array of links.
        if isequal(map_finger.idx, 0) % palm: link idx 0
            finger = 0;
            palmmesh = map_finger.linkmesh;
            scatter3(palmmesh(:,1),palmmesh(:,2),palmmesh(:,3),'MarkerFaceColor','g', 'MarkerEdgeColor', 'g');
            title('Reachability map palm')
            hold on;
        else
            warning('Incorrect data type.');
        end
    else
        %%% plot rmaps of finger links
        nlink = length(map_finger);
        for l = 1:nlink-1 % number of real links (ignore fingertip)
            % If the index of link, l, is in the list of desired link to plot;
            % or if map_idx_f = NaN, the entire finger should be plotted
            finger = f;
            map_link = map_finger{l};

            map_dataset = map_link.linkmesh;

            px = map_dataset(:,1); 
            py = map_dataset(:,2);
            pz = map_dataset(:,3);
            
            % plot alpha shape
            shp = alphaShape(px,py,pz,30);
            plot(shp,'Facecolor',cmap(l,:),'EdgeColor',cmap(l,:),'EdgeAlpha',0.15,'FaceAlpha',0.25);
            titlename = ['Reachable map finger ',num2str(finger)];
            title(titlename);
            hold on;
        end
    end
    
    file_name = ['../database/reachability_map/reachable_map_F',num2str(finger),'.fig'];
    file_name_eps = ['../database/reachability_map/reachable_map_F',num2str(finger),'.eps'];

    grid on;
    axis equal;
    view([-150 30]);
    
    saveas(gcf, file_name);
    saveas(gcf, file_name_eps);
end

end