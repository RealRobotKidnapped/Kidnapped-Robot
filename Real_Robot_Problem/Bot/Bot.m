classdef Bot
    
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
        
        function finish(bot)
            NXT_PlayTone(440, 500);    
        end
        
        function move(bot, distance_cm)
            power_pct = 50;
            distance_meter = distance_cm/100;
            calibrateddistance_meter = (distance_meter + (distance_meter * 0.0004));
            bot.MotorsAB.Power = sign(calibrateddistance_meter)*power_pct;
            bot.MotorsAB.TachoLimit = int32(abs((calibrateddistance_meter/bot.WheelCircumference_m)*360));
            
            bot.MotorsAB.SendToNXT();
            
            bot.MotorsAB.WaitFor();
            
            bot.MotorsAB.Stop();
        end
        
        function turn(bot, angle_rad)
            power_pct = 70;
            angle_deg = toDegrees('radians', angle_rad);
            calibratedAngle_deg = 0.013 * angle_deg + angle_deg;
            tachoLimit = int32((bot.RobotRadius_m/bot.WheelRadius_m)*abs(calibratedAngle_deg));

            bot.MotorA.TachoLimit = int32(tachoLimit);
            bot.MotorB.TachoLimit = int32(tachoLimit);

            if angle_deg > 0            
              bot.MotorA.Power =  power_pct;  %right moved backword
              bot.MotorB.Power = -power_pct;  %left moved forward
            elseif angle_deg == 0 || angle_deg == 2 * pi

            else
              bot.MotorA.Power =-power_pct;  %right moved forward
              bot.MotorB.Power = power_pct;  %left moved backword
            end

            bot.MotorA.SendToNXT();
            bot.MotorB.SendToNXT();

            bot.MotorA.WaitFor();
            bot.MotorB.WaitFor();

            bot.MotorA.Stop();
            bot.MotorB.Stop();
        end
        
        function distances_cm = ultraScan(bot,numOfScans)

            Scans = bot.Scans(-40, 360/numOfScans);
            distances_cm = [ 
                Scans(1, 2); 
                Scans(2, 2); 
                Scans(3, 2); 
                Scans(4, 2); 
                Scans(5, 2); 
                Scans(6, 2);
                Scans(7, 2);
                Scans(8, 2);
                Scans(9, 2)
                ]; 
                
        end
       
        function distances_cm = ultraScanForPath(bot,numOfScans)

            Scans = bot.ScansForPath(-40, 90/numOfScans);
            distances_cm = [ 
                Scans(1, 2); 
                Scans(2, 2); 
                Scans(3, 2); 
                Scans(5, 2);
                Scans(4, 2) 
                
                 
%                 Scans(6, 2);
%                 Scans(7, 2);
%                 Scans(8, 2);
%                 Scans(9, 2)
                ]; 
                
        end
        
        function distances_cm = Scans(bot, power_pct, angle_deg)
            count = 360/angle_deg;
            distances_cm = zeros(count, 2);
            
            % initial position
            totAngle_deg = 319;
            
            for i = 1:count
                distance_cm = GetUltrasonic(SENSOR_4);
                calibrated_sensed_distance = distance_cm + 2;            
                distances_cm(i,:) =  calibrated_sensed_distance;
                
                if i < count
                    rotateSensor(bot, -power_pct, angle_deg);
                end
            end
            rotateSensor(bot, power_pct, totAngle_deg); %reset the scanner
        end
        
        function distanceFromObstacle = scanInFront_cm(bot)
            distanceFromObstacle = GetUltrasonic(SENSOR_4);
        end
       
        function rotateSensor(bot, power_pct, angle_deg)
            bot.MotorC.ActionAtTachoLimit = 'Brake';
            bot.MotorC.Power = power_pct;
            bot.MotorC.SmoothStart = false;
            bot.MotorC.TachoLimit = abs(angle_deg);
                        
            bot.MotorC.SendToNXT();

            bot.MotorC.WaitFor();
            
            bot.MotorC.Stop();
        end

        function distances_cm = ScansForPath(bot, power_pct, angle_deg)
            count = round(90/angle_deg);
            distances_cm = zeros(count, 2);
            
            % initial position
            totAngle_deg = 90;
            
            for i = 1:3
                distance_cm = GetUltrasonic(SENSOR_4);
                calibrated_sensed_distance = distance_cm + 2;            
                distances_cm(i,:) =  calibrated_sensed_distance;
                
                if i < count
                    rotateSensor(bot, -power_pct, angle_deg);
                end
            end
            rotateSensor(bot, power_pct, totAngle_deg); %reset the scanner
            for i = 4:5
                distance_cm = GetUltrasonic(SENSOR_4);
                calibrated_sensed_distance = distance_cm + 2;            
                distances_cm(i,:) =  calibrated_sensed_distance;
                
                if i < count
                    rotateSensor(bot, -power_pct, angle_deg);
                end
            end
            rotateSensor(bot, -power_pct, angle_deg);
        end
        
    end
end

