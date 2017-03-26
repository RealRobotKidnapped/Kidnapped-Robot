classdef Bot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        RobotRadius_m = 0.056;
        WheelRadius_m = 0.029;
        WheelCircumference_m = 2*pi*Bot.WheelRadius_m;
    end
    
    properties
        Handle
        MotorA
        MotorB
        MotorC
        MotorsAB
    end
    
    methods

        %% constructor
        
        function bot = Bot()
            bot.Handle = COM_OpenNXT();
            COM_SetDefaultNXT(bot.Handle);
            
            bot.MotorA = NXTMotor('A');
            bot.MotorA.ActionAtTachoLimit = 'Brake';
            bot.MotorA.SmoothStart = true;
            bot.MotorA.SpeedRegulation = false;
            
            bot.MotorB = NXTMotor('B');
            bot.MotorB.ActionAtTachoLimit = 'Brake';
            bot.MotorB.SmoothStart = true;
            bot.MotorB.SpeedRegulation = false;
            
            bot.MotorC = NXTMotor('C');
            bot.MotorC.ActionAtTachoLimit = 'Brake';
            bot.MotorC.SmoothStart = true;
            bot.MotorC.SpeedRegulation = false;
            
            bot.MotorsAB = NXTMotor('AB');
            bot.MotorsAB.ActionAtTachoLimit = 'Brake';
            bot.MotorA.SmoothStart = true;
            bot.MotorsAB.SpeedRegulation = false;
            
            OpenSwitch(SENSOR_1);
            OpenUltrasonic(SENSOR_4);
            
            NXT_GetBatteryLevel()
        end
        
        %% destructor
        
        function delete(bot)
            bot.MotorA.Stop('off');
            bot.MotorB.Stop('off');
            bot.MotorC.Stop('off');
            bot.MotorsAB.Stop('off');

            CloseSensor(SENSOR_1);
            CloseSensor(SENSOR_4);
            
            COM_CloseNXT(bot.Handle);
        end
        
        %% high-level methods (equivalent to botsim)
        
        function complete(bot)
            NXT_PlayTone(440, 500);    
        end
        
        function move(bot, distance_cm)
           bot.translate(50, distance_cm/100); 
        end
        
        function turn(bot, angle_rad)
            bot.rotate(50, toDegrees('radians', angle_rad));
        end
        
        function distances_cm = ultraScan(bot,numOfScans)

            survey = bot.survey(-50, 360/numOfScans);
            distances_cm = [ 
                survey(1, 2); 
                survey(2, 2); 
                survey(3, 2); 
                survey(4, 2); 
                survey(5, 2); 
                survey(6, 2);
                survey(7, 2);
                survey(8, 2);
                survey(9, 2)
                ]; 
                
        end
        
        %% low-level methods
       
        function distanceFromObstacle = scanInFront_cm(bot)
            distanceFromObstacle = GetUltrasonic(SENSOR_4);
        end
%         function calibratedDistance_cm = getDistance_cm(bot)
%             distance_cm = GetUltrasonic(SENSOR_4);     
%             if (distance_cm > 0 && distance_cm < 200)
%                 calibratedDistance_cm = 1.0078 * distance_cm + 0.2267;
%             else
%                 calibratedDistance_cm = -1;
%             end
%         end

        function touch = getTouch(bot)
            touch = GetSwitch(SENSOR_1);
        end
        
        function rotate(bot, power_pct, angle_deg)
%             calibratedAngle_deg = 0.013 * angle_deg + angle_deg;
%             tachoLimit = int32((bot.RobotRadius_m/bot.WheelRadius_m)*abs(calibratedAngle_deg));
% 
%             bot.MotorA.Power = sign(calibratedAngle_deg)*power_pct;
%             bot.MotorA.TachoLimit = tachoLimit;
%                         
%             bot.MotorA.SendToNXT();
% 
%             bot.MotorB.Power = -sign(calibratedAngle_deg)*power_pct;            
%             bot.MotorB.TachoLimit = tachoLimit;
%             
%             bot.MotorB.SendToNXT();
%             
%             bot.MotorA.WaitFor();
%             bot.MotorB.WaitFor();
%             
%             bot.MotorA.Stop();
%             bot.MotorB.Stop();
%             % -------------------------------------
                % -------------------------------------
                calibratedAngle_deg = 0.013 * angle_deg + angle_deg;
                tachoLimit = int32((bot.RobotRadius_m/bot.WheelRadius_m)*abs(calibratedAngle_deg));
                %--------------------------------------
                bot.MotorA.TachoLimit = int32(tachoLimit);
                bot.MotorB.TachoLimit = int32(tachoLimit);
                %--------------------------------------
                % -------------------------------------    
                if angle_deg > 0
                    %right moved backword
                    %left moved forward
                  bot.MotorA.Power =  power_pct;  
                  bot.MotorB.Power = -power_pct;  
                elseif angle_deg == 0 || angle_deg == 2 * pi
                     
                else
                    %right moved forward
                    %left moved backword
                  bot.MotorA.Power =-power_pct;  
                  bot.MotorB.Power = power_pct;  
                end
                % --------------------------------------
                bot.MotorA.SendToNXT();
                bot.MotorB.SendToNXT();

                bot.MotorA.WaitFor();
                bot.MotorB.WaitFor();

                bot.MotorA.Stop();
                bot.MotorB.Stop();
        end
       
        function rotateDistanceSensor(bot, power_pct, angle_deg)
            bot.MotorC.ActionAtTachoLimit = 'Brake';
%             bot.MotorC.Power = sign(angle_deg)*power_pct;
            bot.MotorC.Power = power_pct;
            bot.MotorC.SmoothStart = false;
            bot.MotorC.TachoLimit = abs(angle_deg);
                        
            bot.MotorC.SendToNXT();

            bot.MotorC.WaitFor();
            
            bot.MotorC.Stop();
        end
        
        % rotate anti-clockwise
        function distances_cm = survey(bot, power_pct, angle_deg)
            count = 360/angle_deg;
            distances_cm = zeros(count, 2);
            
%             % set initial position
              totAngle_deg = 318;

%             rotateDistanceSensor(bot, power_pct, totAngle_deg);
            
            for i = 1:count
                distance_cm = GetUltrasonic(SENSOR_4);
%                 distance_cm = getDistance_cm(bot)
%                 % obtain distance measurement
                distances_cm(i,:) =  distance_cm;
%                 
                if i < count
                    % update position 
                    rotateDistanceSensor(bot, -power_pct, angle_deg);
%                     totAngle_deg = totAngle_deg - angle_deg;
                end
            end
            
            % reset position
            rotateDistanceSensor(bot, power_pct, totAngle_deg);
        end
        
        function translate(bot, power_pct, distance_m)
%             calibratedDistance_m = 1.0469 * distance_m + 0.001326;
            calibratedDistance_m = (distance_m + (distance_m * 0.0004));
%             calibratedDistance_m = (distance_m + (distance_m * 0.0004)+ 0.001);
            bot.MotorsAB.Power = sign(calibratedDistance_m)*power_pct;
%             bot.MotorsAB.Power = power_pct;
            bot.MotorsAB.TachoLimit = int32(abs((calibratedDistance_m/bot.WheelCircumference_m)*360));
%             bot.MotorsAB.TachoLimit = int32(abs((distance_m/bot.WheelCircumference_m)*360));
            
            bot.MotorsAB.SendToNXT();
            
            bot.MotorsAB.WaitFor();
            
            bot.MotorsAB.Stop();
        end
    end
end

