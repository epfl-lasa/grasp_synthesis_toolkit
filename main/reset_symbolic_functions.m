function reset_symbolic_functions()
% function to delete constraint and objective functions built from symbolic
% expressions (should be called anytime the problem definition changes)

if isfile('../database/symbolic_functions/objfun.m')
    % if one file exists, delete all
    delete ../database/symbolic_functions/nonl_ceq.m
    delete ../database/symbolic_functions/nonl_ceq_grad.m
    delete ../database/symbolic_functions/nonl_c.m
    delete ../database/symbolic_functions/nonl_c_grad.m
    delete ../database/symbolic_functions/objfun.m
    delete ../database/symbolic_functions/objfun_grad.m
else
    fprintf('No symbolic functions to delete')
end

