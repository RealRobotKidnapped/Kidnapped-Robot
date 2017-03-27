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
%% Localisation function
tic % starts timer
print = 'particle filtering';
returnedBot = localise(bot,map,target);
pathplanning_Robot(bot,returnedBot,map,target)
bot.finish();
bot.delete();
resultsTime = toc;
