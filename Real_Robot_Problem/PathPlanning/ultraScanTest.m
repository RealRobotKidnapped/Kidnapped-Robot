clear
COM_CloseNXT all
close all
clear all
%COM_CloseNXT all

power_pct = 50;
% ---------------------------------------
bot = Bot();
%----------------------------------------

distance_to_scan = 120;
always_odd = 5;

num_scans = (always_odd - 1);

half_scans = num_scans / 2;

halfAngle = (distance_to_scan / 2);

angle_to_sacn = halfAngle / half_scans;

distances_cm = zeros(always_odd,1);

angle_val = halfAngle;

i = 1;
while ( i <= half_scans)
                if (i == 1)
                    rotateSensor(bot, power_pct, angle_val);
                else
                    rotateSensor(bot, -power_pct, angle_val);
                end
                pause(.1);
                
                dist = GetUltrasonic(SENSOR_4); 
                trying = 1;
                while dist >= 150 && trying <= 5
                    dist = GetUltrasonic(SENSOR_4);
                    dist = dist + 2;  
                    trying = trying + 1;
                end
                
                distances_cm(i) =  dist;
                
                fprintf('angle_val: (%.3f) , distance: (%.3f)  \n', ...
                    angle_val, distances_cm(i)); 
                
                angle_val = angle_val - angle_to_sacn; 
                i = i + 1;
end

rotateSensor(bot, -power_pct, angle_to_sacn); 
pause(.1);
dist = GetUltrasonic(SENSOR_4);
trying = 1;
while dist >= 150 && trying <= 5
    dist = GetUltrasonic(SENSOR_4);
    dist = dist + 2;  
    trying = trying + 1;
end

distances_cm(half_scans + 1) =  dist;

fprintf('angle_val: (%.3f) , distance: (%.3f)  \n', ...
                    angle_to_sacn, distances_cm(half_scans + 1)); 

angle_val = halfAngle;

i = half_scans + 1;

while ( i <= always_odd)
                if (i == half_scans + 1)
                    rotateSensor(bot, -power_pct, angle_val);
                else
                    rotateSensor(bot, power_pct, angle_val);
                end
                pause(.1);
                
                dist = GetUltrasonic(SENSOR_4);
                trying = 1;
                while dist >= 150 && trying <= 5
                    dist = GetUltrasonic(SENSOR_4);
                    dist = dist + 2;  
                    trying = trying + 1;
                end
                
                distances_cm(i) =  dist;

                fprintf('angle_val: (%.3f) , distance: (%.3f)  \n', ...
                    angle_val, distances_cm(i)); 

                angle_val = angle_val - angle_to_sacn;              
                i = i + 1;
end

rotateSensor(bot, power_pct, angle_to_sacn); 

CloseSensor(SENSOR_4);
            
COM_CloseNXT(bot.Handle);

distances_cm
