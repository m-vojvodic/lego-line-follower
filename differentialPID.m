disp('Connecting...')
% brick wifi init
b = Brick('ioType','wifi','wfAddr','192.168.19.4','wfPort',5555,'wfSN','0016534442b3');
% beep to indicate connection successful
b.beep();
disp('Connected!')
disp('@_@')

% the usb chain layer
layer = 0;

% the port number the sensor is attached to
sensor_port = Device.Port2;
sensor_mode = Device.ColReflect;

% default drive power
default_drive_power = -25;
% max drive power
max_drive_power = -60;

% A controls drive
drive_motor = Device.MotorA;
% with this car, negative to go forward...
drive_motor_power = default_power;

% B controls turns
turn_motor = Device.MotorB;
turn_motor_power = 0;

% set motor power
b.outputPower(layer, drive_motor, drive_motor_power)
b.outputPower(layer, turn_motor, turn_motor_power)
% start the motors
b.outputStart(layer, drive_motor)
b.outputStart(layer, turn_motor)

% proportional control variable
P = -2.0;
P_turn = 1;

% integral control variables
I = 0.0;
integ = 0.0;

% derivative control variables
D = -10.0;
deriv = 0.0;

% percent light reflected target value
target = 18.0;

% desired runtime
runtime = 45;

% follow right edge of line
tic
while(toc < runtime)
    % read the color reflected twice
    % calculate error for each
    color = b.inputReadSI(layer, sensor_port, sensor_mode);
    e1 = target - color;
    pause(0.01);
    color = b.inputReadSI(layer, sensor_port, sensor_mode);
    e2 = target - color;
     
    % calculate derivative and integral terms
    deriv = (e2 - e1);
    integ = integ + e2;
    
    % if on a turn, increase motor power proportionally
    if(e2 > 3 || e2 < -3)
        P_turn = 2.5;
    else
        P_turn = 1;
    end
    % if not turning (much)
    if(P_turn == 1)
        % increase speed slowly
        drive_motor_power = drive_motor_power - 1;
        % limit max speed
        if (drive_motor_power < max_drive_power)
           drive_motor_power = max_drive_power;
        end
    % if turning
    else
        % decrease speed quickly
        drive_motor_power = drive_motor_power + 4;
        % limit min speed
        if(drive_motor_power > default_drive_power)
          drive_motor_power = default_drive_power;
        end
        % if speed is fast and want to turn, turn less
        if(drive_motor_power < default_drive_power - 15)
             P_turn = 2.0;
        end
    end
    % calculate turn motor power
    turn_motor_power = (P_turn * P * e2) + (D * deriv) + (I * integ);
    % modify motor powers
    b.outputPower(layer, drive_motor, drive_motor_power);
    b.outputPower(layer, turn_motor, turn_motor_power);
end

% stop all motors
b.outputStopAll();
% beep to signal end of program
b.beep();
disp('Disconnecting...')
% delete brick object
delete(b)
disp('Disconnected!')