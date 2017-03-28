function [Friend] = localise(bot,map,target)

scans = 9;
turnBot = pi/5;
num = 600;
particles(num,1) = BotSim;
variance = 10;

sensorNoise = 1.36; % from robot calibration - 0;%
motionNoise = 0.012; % from robot calibration - 0;%
turningNoise = toRadians('degrees', 2.44); % from robot calibration - 0;%
    
for i = 1:num
    particles(i) = BotSim(map, [ sensorNoise, motionNoise, turningNoise ], 0);  
    particles(i).randomPose(0); 
    particles(i).setScanConfig(particles(i).generateScanConfig(scans));
end

maxNumOfIterations = 30;
n = 0;

while(n < maxNumOfIterations)
    
    n = n+1;
    hold on;
    
    botScan = bot.ultraScan(scans);
    botScan

    scanflag = 0; % flag value to check if the robotscan return legitimate results.
    while(scanflag == 0)
        scanflag = 1;
        for i = 1:length(botScan)
            if (botScan(i,:) < 5 || botScan(i,:) > 130) % illegit value
            scanflag = 0;
            end
        end
        if(scanflag == 0)
            aflag = 0;
            bflag = 0;
             
            for i = 1:length(botScan)
                if (i < 6)
                    if(botScan(i,:) < 16) % if robot has wall in left side
                    aflag = 1;
                    end
                else
                    if(botScan(i,:) < 16) % if robot has wall in right side
                    bflag = 2; 
                    end
                end
            end
            
            if((aflag == 1 && bflag == 2)) % if robot has wall in both left and right side and a wall ahead. In this case it moves back.
                if((botScan(1,:) > 13))
                    bot.move(8); % move back 10 cm
                    bot.turn(turnBot); % turn
                    for i =1:num %for all the particles.
                        particles(i).move(8);
                        particles(i).turn(turnBot); 
                    end
                elseif((botScan(5,:) > 13) && (botScan(6,:) > 13))
                    bot.move(-8); % move back 10 cm
                    bot.turn(-turnBot); % turn
                    for i =1:num %for all the particles.
                        particles(i).move(-8);
                        particles(i).turn(turnBot); 
                    end
                end        
            elseif(aflag == 1)   
                bot.turn(turnBot); % turn clockwise
                for i =1:num %for all the particles.
                    particles(i).turn(turnBot); 
                end
            elseif(bflag == 2)
                bot.turn(-turnBot); %turn anticlockwise
                for i =1:num %for all the particles.
                    particles(i).turn(-turnBot); 
                end
            else
                movedistance = 5; 
                bot.move(movedistance); % move ahead 5 cm
                bot.turn(turnBot);
                for i =1:num %for all the particles.
                    particles(i).move(movedistance); 
                    particles(i).turn(turnBot); 
                end
             end  
            botScan = bot.ultraScan(scans); % Scan again
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
    
    w_distribution = weight./sum(weight); % Distribute weight
    %% Write code for resampling your particles

    [~,BestLocations] = sort(w_distribution ,'descend'); % sort the values in decreasing order and assign to the location
    [~,BestAngles] = sort(w_distribution ,'descend');
    
    locations = zeros(num,2);
    angles = zeros(num,2);
    ranking = zeros(num,1);
        
    for i=1:num
        locations(i,1:2) = particles(BestLocations(i)).getBotPos();
        angles(i) = particles(BestAngles(i)).getBotAng();
        ranking(i) = int16(num * w_distribution(BestLocations(i)));
    end
     
    TopBest = int16(num * .50); % take 50% population back
    
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
            
            loc = orgenal_loc;
            ang = orgenal_ang;
            
            particles( index_of_num).setBotPos(loc);
            particles( index_of_num).setBotAng(ang);
            
            index_of_num = index_of_num + 1;              
            looptimes = looptimes -1;
        end
        index_location = index_location + 1;
    end
    
    figure(1)
        hold off; 
        particles(1).drawMap(); 
        for i =1:num
            particles(i).drawBot(3); 
        end
        drawnow;
        
    %particle's positions and angles
    ang = zeros(num,1);
    pos = zeros(num, 2);   
   
    for i = 1:num
        pos(i,:) = particles(i).getBotPos();
        ang(i)=particles(i).getBotAng();
    end
    
    %% Write code to check for convergence   
    if std(pos) < 5 % convergence threshold
        
        for i = 1:length(botScan)
            if (i < 6)
                if(botScan(i,:) < 16) % if robot has wall in left side
                aflag = 1;
                end
            else
                if(botScan(i,:) < 16) % if robot has wall in right side
                bflag = 2; 
                end
            end
        end

        if((aflag == 1 && bflag == 2)) % if robot has wall in both left and right side and a wall ahead. In this case it moves back.
                if((botScan(1,:) > 13))
                    bot.move(8); % move back 10 cm
                    bot.turn(turnBot); % turn
                    for i =1:num %for all the particles.
                        particles(i).move(8);
                        particles(i).turn(turnBot); 
                    end
                elseif((botScan(5,:) > 13) && (botScan(6,:) > 13))
                    bot.move(-8); % move back 10 cm
                    bot.turn(-turnBot); % turn
                    for i =1:num %for all the particles.
                        particles(i).move(-8);
                        particles(i).turn(turnBot); 
                    end
                end              
        elseif(aflag == 1)   
            bot.turn(turnBot); % turn clockwise
            bot.move(-8);
            for i =1:num %for all the particles.
                particles(i).turn(turnBot); 
                particles(i).move(-8);
            end
        elseif(bflag == 2)
            bot.turn(-turnBot); %turn anticlockwise
            bot.move(-8);
            for i =1:num %for all the particles.
                particles(i).turn(-turnBot); 
                particles(i).move(-8);
            end
        else
            movedistance = 5; 
            bot.move(movedistance); % move ahead 5 cm
            bot.turn(turnBot);
            for i =1:num %for all the particles.
                particles(i).move(movedistance); 
                particles(i).turn(turnBot); 
            end
        end
        
        Friend_mean = BotSim(map);
        Friend_mean.setScanConfig(Friend_mean.generateScanConfig(scans));
        Friend_mean.setBotAng(mean(ang));
        Friend_mean.setBotPos(mean(pos));

        Friend_mode = BotSim(map); 
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
        break; %on convergence of particles
    end
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    mutation_rate=0.01; 
    
    for i=1:mutation_rate*num
        particles(randi(num)).randomPose(0);
    end
    

    %% Write code to decide how to move next
    aflag = 0;
    bflag = 0;
    turn = pi/4; %turn 45 degree
    
    %Code to avoid collision
    for i = 1:length(botScan)
        if (i < 6)
            if(botScan(i,:) < 20)
            aflag = 1;
            end
        else
            if(botScan(i,:) < 20)
            bflag = 2;
            end
        end
    end

    if((aflag == 1 && bflag == 2)) % if robot has wall in both left and right side and a wall ahead. In this case it moves back.
            if((botScan(1,:) > 15))
                bot.move(8); % move back 10 cm
                bot.turn(turnBot); % turn
                for i =1:num %for all the particles.
                    particles(i).move(8);
                    particles(i).turn(turnBot); 
                end
            elseif((botScan(5,:) > 15) && (botScan(6,:) > 15))
                bot.move(-8); % move back 10 cm
                bot.turn(-turnBot); % turn
                for i =1:num %for all the particles.
                    particles(i).move(-8);
                    particles(i).turn(turnBot); 
                end
            end   
    elseif(aflag == 1)   
        bot.turn(turn);
        bot.move(-8);
        for i =1:num %for all the particles.
            particles(i).turn(turn); 
            particles(i).move(-10);
        end
    elseif(bflag == 2)
        bot.turn(-turn);
        bot.move(-8);
        for i =1:num %for all the particles.
            particles(i).turn(-turn); 
            particles(i).move(-10);
        end
    else
        movedistance = 5;
        bot.move(movedistance);
        bot.turn(turn);
        for i =1:num %for all the particles.
            particles(i).move(movedistance); 
            particles(i).turn(turn); 
        end
    end     
        
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    figure(1)
        hold off; 
        particles(1).drawMap(); 
        for i =1:num
            particles(i).drawBot(3); 
        end
        plot(target(1),target(2),'Marker','o','Color','g');
        drawnow; 
end

    botScan = bot.ultraScan(scans);        
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
    
    [min_diff_mean, min_pos_mean] = min(difference_mean);
    Friend_mean.setBotAng(min_pos_mean*pi/180); 

    [min_diff_mode, min_pos_mode]=min(difference_mode);
    Friend_mode.setBotAng(min_pos_mode*pi/180);


    if min_diff_mean < min_diff_mode 
        Friend = Friend_mean;
    else
        Friend = Friend_mode;
    end
    figure(2)
        hold off; 
        particles(1).drawMap(); 
        for i =1:num
            particles(i).drawBot(3); 
        end
        Friend.drawBot(30, 'r');
        plot(target(1),target(2),'Marker','o','Color','g');
        drawnow;
end
