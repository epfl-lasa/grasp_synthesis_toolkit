function reset_symbolic_functions()
% function to delete constraint and objective functions built from symbolic
% expressions (should be called anytime the problem definition changes)

functions_exist = false;

% if one file exists, delete all
if isfile('../database/symbolic_functions/nonl_c.m')
    delete ../database/symbolic_functions/nonl_c.m
    functions_exist = true;
end
if isfile('../database/symbolic_functions/nonl_c_grad.m')
    delete ../database/symbolic_functions/nonl_c_grad.m
    functions_exist = true;
end

% nonlinear equality constraints
if isfile('../database/symbolic_functions/nonl_ceq.m')
    delete ../database/symbolic_functions/nonl_ceq.m
    functions_exist = true;
end

if isfile('../database/symbolic_functions/nonl_ceq_grad.m')
    delete ../database/symbolic_functions/nonl_ceq_grad.m
    functions_exist = true;
end

% objective function
if isfile('../database/symbolic_functions/objfun.m')
    delete ../database/symbolic_functions/objfun.m
    functions_exist = true;
end
if isfile('../database/symbolic_functions/objfun_grad.m')
    delete ../database/symbolic_functions/objfun_grad.m
    functions_exist = true;
end

if functions_exist
    fprintf('\nDeleted objective and constraint functions\n')
else
    fprintf('\nNo objective and constraint to delete\n')
end

