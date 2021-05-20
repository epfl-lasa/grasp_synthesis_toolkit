function dist = distToLink(link, point)
L = link.L;
x1 = link.symbolic.HT_next(1:3,4);
x0 = link.symbolic.HT_this(1:3,4);

cross_prod = cross(point - x0, x1 - x0);
dist = norm(cross_prod)/L;

end