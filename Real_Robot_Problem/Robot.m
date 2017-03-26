close all
clear all
clc

%% Initialise the map and size of the robot
map = [0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];% representing external boundaries

botSim = BotSim(map,[0,0,0], 0);  % sets up a botSim object with a map, and debug mode on.
botSim.drawMap();
drawnow;
target = botSim.getRndPtInMap(10);  % gets random endpoint / target.

bot = Bot();
%% Localisation function using Particle Filtering to estimate the current 
% location of the robot
tic % starts timer
print = 'particle filtering';
% returnedBot = localise(bot,map,target);
returnedBot = DistanceCheck(bot,map,target);
returnedBot.getBotPos()
returnedBot.getBotAng()
pathplanning_Robot(returnedBot,botSim,map,target)
bot.complete();
bot.delete();
resultsTime = toc;

% resultsDis = distance(target, returnedBot.getBotPos())