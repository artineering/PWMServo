classdef PWMServo < handle
    % PCA9685 Interface for the PCA9685 PWM Servo Driver
    % PCA9685 Pwm Servo Driver is assumed to connected to the I2C bus on
    % address 0x40.
    
    properties
        controller
        i2cAddress
        i2cDevice
        freq
        LED0_ON_L = 0x06
        LED0_ON_H = 0x07
        LED0_OFF_L = 0x08
        LED0_OFF_H = 0x09
        servoMin = 150
        servoMax = 600
    end

    methods
        function obj = PWMServo(controller,i2cAddress)
            obj.controller = controller;
            obj.i2cAddress = i2cAddress;
            obj.i2cDevice = device(obj.controller, 'I2CAddress', obj.i2cAddress);

            % Set all PWM to 0
            write(obj.i2cDevice, [hex2dec('FA') hex2dec('0')]);
            write(obj.i2cDevice, [hex2dec('FB') hex2dec('0')]);
            write(obj.i2cDevice, [hex2dec('FC') hex2dec('0')]);
            write(obj.i2cDevice, [hex2dec('FD') hex2dec('0')]);
            
            write(obj.i2cDevice,[hex2dec('01') hex2dec('04')]);
            write(obj.i2cDevice,[hex2dec('00') hex2dec('01')]);

            % Initialize mode register.
            mode1 = readRegister(obj.i2cDevice, hex2dec('00'));
            mode1 = mode1 & (bitcmp(hex2dec('10'),'int8'));
            write(obj.i2cDevice, [hex2dec('FA') mode1]);

            pause(0.05);
        end

        function setServoMin(obj, min)
            obj.servoMin = min;
        end

        function setServoMax(obj, max)
            obj.servoMax = max;
        end

        function setFrequency(obj, freq)
            obj.freq = freq;
            preScale = floor((6103 / freq) - 0.5);
            oldmode = readRegister(obj.i2cDevice, hex2dec('00'));
            newmode = bitor((bitand(oldmode,hex2dec('7F'))),hex2dec('10'));
            
            write(obj.i2cDevice,[hex2dec('00') newmode]); % 0x11 OK
            write(obj.i2cDevice,[hex2dec('FE') preScale]); % 0x65 OK
            write(obj.i2cDevice,[hex2dec('00') oldmode]); % 0x01 OK
            pause(0.05);
            write(obj.i2cDevice,[hex2dec('00') bitor(oldmode,hex2dec('80'))]);

            pause(0.05);
        end
        
        function setServoPulse(obj, channel, pulse)
            onLAddr = obj.LED0_ON_L + 4 * channel;
            onHAddr = obj.LED0_ON_H + 4 * channel;
            offLAddr = obj.LED0_OFF_L + 4 * channel;
            offHAddr = obj.LED0_OFF_H + 4 * channel;

            write(obj.i2cDevice,[onLAddr 0]);
            write(obj.i2cDevice,[onHAddr 0]);
            write(obj.i2cDevice,[offLAddr bitand(pulse,hex2dec('FF'))]);
            write(obj.i2cDevice,[offHAddr bitshift(pulse,-8)]);

            pause(0.05);
        end
    end
end

