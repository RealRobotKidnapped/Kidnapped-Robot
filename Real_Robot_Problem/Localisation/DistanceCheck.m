function [bot] = DistanceCheck(bot,map,target)

scans = 9;
turnBot = pi/5;
num = 600;
particles(num,1) = BotSim;
variance = 10;

for i = 1:num
    particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(particles(i).generateScanConfig(scans));
end

maxNumOfIterations = 30;
n = 0;
while(n < maxNumOfIterations)
    n = n+1;
    hold on;
    botScan = bot.ultraScan(scans);
    botScan

    scanflag = 0;
    while(scanflag == 0)
        scanflag = 1;
        for i = 1:length(botScan)
            if (botScan(i,:) < 5 || botScan(i,:) > 130)
            scanflag = 0;
            end
        end
        if(scanflag == 0)
            aflag = 0;
            bflag = 0;
             
            for i = 1:length(botScan)
                if (i < 6)
                    if(botScan(i,:) < 13)
                    aflag = 1;
                    end
                else
                    if(botScan(i,:) < 13)
                    bflag = 2;
                    end
                end
            end
            
            if((botScan(1,:) < 13) || (aflag == 1 && bflag == 2))
                bot.move(-10);
                bot.turn(-turnBot);
                for i =1:num %for all the particles.
                    particles(i).move(-10);
                    particles(i).turn(-turnBot); 
                end
            elseif(aflag == 1)   
                bot.turn(turnBot);
                for i =1:num %for all the particles.
                    particles(i).turn(turnBot); 
                end
            elseif(bflag == 2)
                bot.turn(-turnBot);
                for i =1:num %for all the particles.
                    particles(i).turn(-turnBot); 
                end
            else
                movedistance = 5;
                bot.move(movedistance);
                bot.turn(turnBot);
                for i =1:num %for all the particles.
                    particles(i).move(movedistance); 
                    particles(i).turn(turnBot); 
                end
             end  
            botScan = bot.ultraScan(scans);
            botScan
        end
    end

    weight = zeros(num,1);
    sub = zeros(scans,num);
    p_w = zeros(scans,1);
    
    for i=1:num
        if particles(i).insideMap() == 0
            particles(i).randomPose(10);
        end
        
        [dist , crossPnt] = particles(i).ultraScan();
        distCPointMatrix = dist;
        
        for j=1:scans
            %% Write code for scoring your particles  
            temp = circshift(distCPointMatrix,j);
            sub(j,i) = sqrt(sum((temp-botScan).^2));
            p_w(j) = (1/sqrt(2*pi*variance))*exp(-((sub(j,i))^2/(2*variance)));
        end
        [max_weight, max_position] = max(p_w);
        %particle orintation
        particles(i).setBotAng(mod((particles(i).getBotAng() + max_position*2*pi/scans), 2*pi)); 
        weight(i) = max_weight;
    end 
    
    w_distribution = weight./sum(weight);
    %% Write code for resampling your particles
        
%     for i = 1:num 
%         j = find(rand() <= cumsum(w_distribution),1);
%         particles(i).setBotPos(particles(j).getBotPos());
%         particles(i).setBotAng(particles(j).getBotAng());
%     end


    [~,BestLocations] = sort(w_distribution ,'descend');
    [~,BestAngles] = sort(w_distribution ,'descend');
    
    locations = zeros(num,2);
    angles = zeros(num,2);
    ranking = zeros(num,1);
        
    for i=1:num
        locations(i,1:2) = particles(BestLocations(i)).getBotPos();
        angles(i) = particles(BestAngles(i)).getBotAng();
        ranking(i) = int16(num * w_distribution(BestLocations(i)));
    end
     
    TopBest = int16(num * .50);
    for i=1:TopBest 
        particles(i).setBotPos(locations(i,1:2));
        particles(i).setBotAng(angles(i));
    end
    
    index_of_num = TopBest+1;
                
    index_location = 1;
    
    while index_of_num < num
        
        if index_location == TopBest
             index_location = 1;
        end
        
        orgenal_loc = locations(index_location,1:2);
        orgenal_ang = angles(index_location);
        
        looptimes = ranking(index_location);
        
        while looptimes > 0 && index_of_num < num
%       Gaussian destitution representation provides an approximation of the 
%       error and the belief measurement of each particle given the scan measurements 
%       of the robot            
%             noiseLoc = 1 + w_distribution(index_location) * randn(1,1);
%             noiseAng = 1 + w_distribution(index_location) *(pi/180).*randn(1,1);
%             
%             loc = orgenal_loc + noiseLoc;
%             ang = orgenal_ang + noiseAng;
            
            loc = orgenal_loc;
            ang = orgenal_ang;
            
            particles( index_of_num).setBotPos(loc);
            particles( index_of_num).setBotAng(ang);
            
            index_of_num = index_of_num + 1;
                   
            looptimes = looptimes -1;
        end
        index_location = index_location + 1;
    end
  
    
    
    figure(3)
        hold off; %the drawMap() function will clear the drawing when hold is off
        particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the botsn
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
        
    %particle's positions and angles
    ang = zeros(num,1);
    pos = zeros(num, 2);   
   
    for i = 1:num
        pos(i,:) = particles(i).getBotPos();
        ang(i)=particles(i).getBotAng();
    end
    
    Friend_mean = BotSim(map);
    Friend_mean.setScanConfig(Friend_mean.generateScanConfig(scans));
    Friend_mean.setBotAng(mean(ang));
    Friend_mean.setBotPos(mean(pos));
    
    %Set the mode estimate
    Friend_mode = BotSim(map); %, [ sensorNoise, motionNoise, turningNoise ], 0);
    Friend_mode.setScanConfig(Friend_mode.generateScanConfig(scans));
    Friend_mode.setBotPos(mode(round(pos)));
    Friend_mode.setBotAng(mean(ang));
    
    Friend_meanScan = Friend_mean.ultraScan();
    Friend_modeScan = Friend_mode.ultraScan();
    diff_mean = norm(Friend_meanScan-botScan);
    diff_mode = norm(Friend_modeScan-botScan);

    if diff_mean < diff_mode
        Friend_mean.getBotPos()
    else
        Friend_mode.getBotPos()
    end
%         figure(3)
%         hold off; %the drawMap() function will clear the drawing when hold is off
%         particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the botsn
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
%         Friend_mean.drawBot(30, 'r');
%         drawnow;
       
     %% Write code to check for convergence   

    if std(pos) < 5 % convergence threshold
        break; %particles have converged
    end
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    mutation_rate=0.01;
    
    for i=1:mutation_rate*num
        particles(randi(num)).randomPose(0);
    end
    
%     figure(3)
%         hold off; %the drawMap() function will clear the drawing when hold is off
%         particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the botsn
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
%         Friend_mean.drawBot(30, 'r');
%         drawnow;
        
     %% Write code to decide how to move next
    aflag = 0;
    bflag = 0;
    turn = pi/4;
%     distanceToObstacle = bot.scanInFront_cm();
    
    for i = 1:length(botScan)
        if (i < 6)
            if(botScan(i,:) < 13)
            aflag = 1;
            end
        else
            if(botScan(i,:) < 13)
            bflag = 2;
            end
        end
    end

    if((botScan(1,:) < 18) || (aflag == 1 && bflag == 2))
        bot.move(-14);
        bot.turn(-turn);
        for i =1:num %for all the particles.
            particles(i).move(-14);
            particles(i).turn(-turn); 
        end
    elseif(aflag == 1)   
        bot.turn(turn);
        for i =1:num %for all the particles.
            particles(i).turn(turn); 
        end
    elseif(bflag == 2)
        bot.turn(-turn);
        for i =1:num %for all the particles.
            particles(i).turn(-turn); 
        end
    else
%         if (distanceToObstacle > 30)
%         movedistance = distanceToObstacle*0.3; % potentially use small fixed increment
%         else
        movedistance = 5;
%         end
        bot.move(movedistance);
        bot.turn(turn);
        for i =1:num %for all the particles.
            particles(i).move(movedistance); 
            particles(i).turn(turn); 
        end
    end     
    
     
    
% %     bot.move(move); %move the real robot. These movements are recorded for marking 
%     bot.turn(turn);
%     
%     for i =1:num %for all the particles.
% %         particles(i).move(move); %move the particle in the same way as the real robot
%         particles(i).turn(turn); %turn the particle in the same way as the real robot
%     end
%     
%     figure(3)
%         hold off; %the drawMap() function will clear the drawing when hold is off
%         particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the botsn
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
%         Friend_mean.drawBot(30, 'r');
%         drawnow;
        
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    figure(3)
    hold off; %the drawMap() function will clear the drawing when hold is off
    particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    for i =1:num
        particles(i).drawBot(3); %draw particle with line length 3 and default color
    end
    plot(target(1),target(2),'Marker','o','Color','g');
    drawnow; 
end

%     botScan = bot.ultraScan(scans);
    difference_mean= zeros(360,1);
    difference_mode= zeros(360,1);
    
    for i=1:360   
    Friend_meanScan = Friend_mean.ultraScan();
    Friend_modeScan = Friend_mode.ultraScan();
    difference_mean(i) = norm(Friend_meanScan - botScan);
    difference_mode(i) = norm(Friend_modeScan - botScan);
    Friend_mean.setBotAng(i*pi/180);
    Friend_mode.setBotAng(i*pi/180);
    end
    
    %find the best orientation for the mean estimate
    [min_diff_mean, min_pos_mean] = min(difference_mean);
    Friend_mean.setBotAng(min_pos_mean*pi/180); 

    %find the best orientation for the mode estimate
    [min_diff_mode, min_pos_mode]=min(difference_mode);
    Friend_mode.setBotAng(min_pos_mode*pi/180);


    if min_diff_mean < min_diff_mode %pick best from mean or mode estimates
        Friend = Friend_mean;
    else
        Friend = Friend_mode;
    end
    figure(3)
        hold off; %the drawMap() function will clear the drawing when hold is off
        particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the botsn
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        Friend.drawBot(30, 'r');
        drawnow;
end
