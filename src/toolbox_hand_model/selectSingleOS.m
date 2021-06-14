function osCandList = selectSingleOS(OS, tgt_pair, object)
    if nargin < 3
        fprintf("[OS selection] Returning all good opposition spaces\n");
        return_all = true;
    else
        fprintf("[OS selection] Returning OS with maximal margin\n");
        const = load('problem_config.mat','f_mu');      % load constant values from config files
        hand_radius = 14;
        d_min = 2*object.radius*cos(atan(const.f_mu)) + 2*hand_radius*cos(atan(const.f_mu));
        d_max = 2*object.radius + 2* hand_radius;
        best_margin = 0;
        return_all = false;
    end
    
    if nargin < 2
        warning('desired os pair not given.');
        tgt_pair = {[1,4],[2,4]}; % desired pair
    end
    
    osCandList = {}; % list of testing os pairs in Opposition Space
    for i = 1:numel(OS) % around 90 Opposition Space in total
        os_info = OS{i}.os_info;
        os_dist = OS{i}.os_dist;
        if (isequal(os_info{1},tgt_pair{1}) && isequal(os_info{2},tgt_pair{2})) ||...
            (isequal(os_info{1},tgt_pair{2}) && isequal(os_info{2},tgt_pair{1})) % symmetric
            fprintf("OS min dist: %5.1f,\t max dist: %5.1f\n", OS{i}.os_dist(1), OS{i}.os_dist(2));
            if return_all
                osCandList{end+1} = OS{i}; % this ith os_info exists in the testing pair, so register the index
            else
                margin = min([d_max - os_dist(1), os_dist(2) - d_min]);
                if margin > best_margin
                    osCandList{1} = OS{i};
                end
            end
        end
    end

    if isempty(osCandList)
        warning('OS to fit the object does not exist.');
    end
end