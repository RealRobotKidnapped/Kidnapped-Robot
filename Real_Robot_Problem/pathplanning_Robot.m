function [bot] = pathplanning_Robot(bot,botSim,map,target)
%% Path Planning

limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map

modifiedMap = map;

%% Plot the chosen path

stPoint = [30 85];
botAng = 0;

h = round(dims(1));

fprintf('Starting Point: %.3f, %.3f \n', stPoint);
fprintf('Starting Angle: %.3f \n', botAng);
%fprintf('Probabilistic Road Points \n');

particle = BotSim(modifiedMap);


probRoadPoints = zeros(h,2);
probRoadPoints(:,1) = limsMin(1) + (limsMax(1) - limsMin(1))*rand(h,1);
probRoadPoints(:,2) = limsMin(2) + (limsMax(2) - limsMin(2))*rand(h,1);

inside = botSim.pointInsideMap(probRoadPoints);
%disp(inside);

RI = 3;
phew = [];

%starting point is 1st point, target is 2nd point
phew(1,:) = stPoint;
phew(2,:) = target;

 for i = 1:h
     if inside(i) == 1
        %plot(probRoadPoints(i,1),probRoadPoints(i,2),'*');
        phew(RI,:) = probRoadPoints(i,:);
        RI = RI+1;
     end 
 end
 
pewIndex = 1;

 

withinBounds = ones(RI,1);

 

 for i = 1:(RI-1)

     

     %disp(i);

     

     particle.setBotPos(phew(i,:));

     particle.generateScanConfig(6);

     

     boundary = particle.ultraScan();

     %disp(boundary);

     

     for j = 1:6

         if boundary(j,1) <= 20 && i ~= 1 && i ~= 2

             withinBounds(i,1) = 0;   

         end

     end

     if withinBounds(i,1) == 1

         pew(pewIndex,:) = phew(i,:);

         pewIndex = pewIndex + 1;

         %disp(pewIndex)

     end

 end 

 disp('this is pew');

 disp(pew);

 

z = size(pew,1);
 
%disp(pew);

z = size(pew,1);
%disp(z);


STindex = 1;

pairs = [];

for i = 1:z
    stP = pew(i,:);
    %fprintf('start: %.3f, %.3f \n',stP);
    particle.setBotPos(stP);
    for j = 1:z
        target = pew(j,:);
        %fprintf('target: %.3f, %.3f \n',target);
        ty = (target(2)-stP(2));
        tx = (target(1)-stP(1));
        %fprintf('toa: %.3f',toa);
        angle = atan(ty/tx);
        if((ty < 0 && tx < 0)||(ty > 0 && tx <0))
            angle = angle + pi;
        end
        
        %fprintf('angle: %.3f \n',angle);
        particle.setBotAng(angle);
        particle.setScanConfig(botSim.generateScanConfig(1));
        sc = particle.ultraScan();
        %fprintf('scan: %.3f \n',sc);
        dist = sqrt((target(1)-stP(1))^2 + (target(2)-stP(2))^2);
        
        numPoints = 50;
        onLineDist = dist/numPoints;
        
        pointsOnLine(numPoints,1) = BotSim;
        
        POLresults = ones(numPoints,1);
        
        for k = 1:numPoints-2
            pointsOnLine(k) = BotSim(modifiedMap);
            pointsOnLine(k).setBotPos(particle.getBotPos());
            pointsOnLine(k).setBotAng(particle.getBotAng());
            pointsOnLine(k).move(onLineDist*(k+1));
            pointsOnLine(k).setScanConfig(botSim.generateScanConfig(6));
            POLscan = pointsOnLine(k).ultraScan();
            for l = 1:6
                if POLscan(l) < 10
                    POLresults(k) = 0;
                end
            end
        end
        Lia = ismember(0,POLresults);
%         if sc > dist && (i == 1 || j == 2)
%             pairs(STindex,:) = [i j];
%             STindex = STindex +1;
%         else 
        if sc > dist && Lia == 0 
            pairs(STindex,:) = [i j];
            STindex = STindex +1;
           
        end
    end
end

pre = sort(pairs,2);
uPairs = unique(pre,'rows');

%disp(uPairs);


s = uPairs(:,1);
t = uPairs(:,2);
% t(t>z) = z;
p1 = pew(s,:);
p2 = pew(t,:);
weights = sqrt((p2(1)-p1(1))^2 + (p2(2)-p1(2))^2);
%disp(weights);
G = graph(s,t,weights);


p = plot(G,'XData',pew(:,1), 'YData',pew(:,2),'EdgeLabel',G.Edges.Weight);

path = shortestpath(G,1,2);
highlight(p,path);

fprintf('Path taken: ');
disp(path);
tests = numel(path);
cpath = pew(path,:);
%fprintf('size cpath: %.3f', tests);
%fprintf('cpath: %',cpath);
curPoint = cpath(1,:);
curAng = botAng;


for i = 2:tests
    plot(curPoint(1,1),curPoint(1,2),'*','MarkerSize', 20);
     nextPoint = cpath(i,:);
     by = (nextPoint(2)-curPoint(2));
     bx = (nextPoint(1)-curPoint(1));
     
     bAng = atan(by/bx);
     %disp(bAng);
    
     if(by > 0 && bx < 0)
            bAng = bAng + pi;
     end
     if (by < 0 && bx < 0)
        bAng = bAng -pi; 
     end

%       if((by < 0 && bx < 0)||(by>0 && bx < 0))
%           bAng = bAng + pi;
%       end
     turn = bAng-curAng;
     
     disp(turn);
    
     %fprintf('turn: %.3f \n', turn);
     bot.turn(turn);
     disp('it hits here');
     distance = sqrt((nextPoint(1)-curPoint(1))^2 + (nextPoint(2)-curPoint(2))^2);
     %fprintf('distance: %.3f \n', distance);
     extrad = distance*(0.5/10);
     bot.move(distance-extrad);
     curAng = bAng;
     curPoint = cpath(i,:);
end

%% Drawing
%only draw if you are in debug mode or it will be slow during marking
if botSim.debug()
    figure(1)
    hold off; %the drawMap() function will clear the drawing when hold is off
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    botSim.drawBot(30,'g'); %draw robot with line length 30 and green
    drawnow;
end

