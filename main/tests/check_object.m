% check object definition

function [error] =  check_object(object)

error = false;
if strcmp(object.type, 'cyl')
    %% check the object normal
    q = compact(object.quat); % store quaternion in array form
    sym_n = subs(object.sym.n, {'p1','p2','p3','p4'},{q(1),q(2),q(3),q(4)});
    if (norm(object.n - sym_n,2) <= 0.001)
        fprintf('Object normal ok\n')
    else
        fprintf('Object normal and symbolic expression are not equivalent\n')
        error = true;
    end
    
    %% check the object axis points
    ctr = object.ctr;
    for i=1:size(object.axPtArray,2)
        pt = object.axPtArray(:,i);
        pt_sym = subs(object.sym.axisPtArray(:,i),{'x','y','z','p1','p2','p3','p4'},{ctr(1),ctr(2),ctr(3),q(1),q(2),q(3),q(4)});
        if (norm(pt_sym - pt,2) <= 1e-3)
            fprintf('Object axis point ok\n');
        else
            error = true;
            fprintf('Object axis point wrong\n');  
        end
    end
end

end