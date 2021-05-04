function [hand] = reset_hand()
% Reload the hand before relaunching the optimization
    models = load('models.mat');
    hand = models.hand;
    fprintf('\n[1] Hand model reloaded.\n');
end

