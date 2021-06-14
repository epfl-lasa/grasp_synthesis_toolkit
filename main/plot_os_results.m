% analyze data of opposition space

success_map = nan(13);

for k=1:numel(os_list)
    FL1 = os_list{k}.os_info{1};
    FL2 = os_list{k}.os_info{2};
    
    finger1 = FL1(1);
    link1 = FL1(2);
    
    finger2 = FL2(1);
    link2 = FL2(2);
    
    % find the correct column
    if finger1 == 0
        i = 13;
    else
        if finger1 == 1
            i = link1-1;
        else
            i = 3*(finger1-1) + link1 - 1;
        end
    end
    
    if finger2 == 0
        j = 13;
    else
        if finger2 == 1
            j = link2-1;
        else
            j = 3*(finger2-1) + link2 - 1;
        end
    end
    fprintf("F%dL%d \t %d \t\t  F%dL%d \t %d\n", finger1, link1, i, finger2, link2, j);
    
    success = 0;
    for n=1:numel(successful_os)
        tmp_os1 = successful_os{n}{1};
        tmp_os2 = successful_os{n}{2};
        tmp_F1 = tmp_os1(1);
        tmp_L1 = tmp_os1(2);
        
        tmp_F2 = tmp_os2(1);
        tmp_L2 = tmp_os2(2);
        if (tmp_F1 == finger1) && (tmp_L1 == link1)
            if (tmp_F2 == finger2) && (tmp_L2 == link2)
                success = 1;
            end
        end
    end
    success_map(i,j) = success;
            
end

category_list = {'1-2','1-3','1-4','2-2','2-3','2-4','3-2','3-3','3-4','4-2','4-3','4-4','P'}

col_bar = [1,0,0;0.8,0,0.2;0.6,0,0.4;0.4,0,0.6;0.2,0,0.8;0,0,1];

figure;
heatmap(success_map,...
    'XData',category_list,...
    'YData',category_list,...
    'MissingDataColor', 'w',...
    'Colormap',col_bar,...
    'ColorLimits',[0 1],...
    'CellLabelColor','none');
colorbar off;
title(['Successful grasps (object radius ', num2str(15),')']);
xlabel('Finger-Link pair');
ylabel('Finger-Link pair');
set(gca, 'FontSize', 14);
    