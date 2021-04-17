function [x] = evaluate_result(hand, object, os_pair, opt_soln)
ctr = object.ctr;
p= compact(object.quat);
ptArraySym = subs(object.sym.axisPtArray,{'x','y','z','p1','p2','p3','p4'},{ctr(1),ctr(2),ctr(3),p(1),p(2),p(3),p(4)});
ptArrayNum = object.axPtArray;
nPoints = size(ptArrayNum,2);
dist = zeros(1,nPoints);

X_sol = opt_soln.X_sol;
mu = X_sol(8:9);
for i=1:nPoints
    dist(i) = norm(ptArrayNum(:,i)-ptArraySym(:,i),2);
end
disp(dist);

os1 = os_pair{1};
os2 = os_pair{2};
idxF = [os1(1),os2(1)];
idxL = [os1(2),os2(2)];

dist_cp = zeros(1,2);
for i=1:2
    dist_cp(i) = norm(hand.F{idxF(i)}.Link{idxL(i)}.contact.p - object.cp(:,i));
end
disp(dist_cp);

cpSym = zeros(3,2);
fprintf("Symbolic values:\n");
cpSym(:,1) = eval(subs(object.sym.cpProj(:,1),{'x','y','z','p1','p2','p3','p4','mu1'},{ctr(1),ctr(2),ctr(3),p(1),p(2),p(3),p(4),mu(1)}));
cpSym(:,2) = eval(subs(object.sym.cpProj(:,2),{'x','y','z','p1','p2','p3','p4','mu2'},{ctr(1),ctr(2),ctr(3),p(1),p(2),p(3),p(4),mu(2)}));
disp(cpSym)
fprintf("Numerical values:\n");
disp(object.cp)

% compute distances
distCpPoints = zeros(1,2);
distCpPoints(1) = norm(object.cp(:,1) - cpSym);
distCpPoints(2) = norm(object.cp(:,2) - cpSym);
fprintf("Distance between contact points (numerical vs symbolic):\n ");
disp(distCpPoints)

end