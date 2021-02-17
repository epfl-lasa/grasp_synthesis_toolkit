function mySGplotPalm_allegro(hand,radius,nps)

if nargin == 1
    % default settings
    radius = 5/1000;
    nps = 5;
elseif nargin == 2
    nps = 5;
end

F = hand.F;
n = hand.n;
basepoints = zeros(3,n+1);

for i= 1:length(F)
    basepoints(:,i)= F{i}.base(1:3,4);
end

c = length(F);
if abs(hand.F{2}.base(1,4)-hand.F{c}.base(1,4)) > abs(hand.F{2}.base(2,4)-hand.F{c}.base(2,4))
    puntodx = [hand.F{c}.base(1,4)
        hand.F{1}.base(2,4)
        hand.F{c}.base(3,4)];
else
    puntodx = [hand.F{1}.base(1,4)
        hand.F{c}.base(2,4)
        hand.F{c}.base(3,4)];
end

basepoints(:,n+1) = puntodx;

cp = basepoints';

co = mean(cp);

for i = 1:size(cp,1)
    ncp(i,:) = (co - cp(i,:))/norm(co - cp(i,:));
end

X = cp;

for i = 1:size(cp,1)
    [filletx,fillety,filletz] = sphere(nps);
    spherecenter = cp(i,:) + radius*ncp(i,:);
    for j = 1:nps+1
        for k = 1:nps+1
            fillett = radius*[filletx(j,k) fillety(j,k) filletz(j,k)]+spherecenter;
            X = [X;fillett];
        end
    end
end

k = convhulln(X);
trisurf(k,X(:,1),X(:,2),X(:,3));
colormap([0.3 0.3 0.3]);
hold on