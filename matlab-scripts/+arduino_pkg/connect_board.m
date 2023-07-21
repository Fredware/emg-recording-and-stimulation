function [board, connected] = connect_board(varargin)
% This function sets up the communication between matlab and the arduino,
% for one EMG channel input.
% uno is a serial communication MATLAB object that is used to get the data
% from the arduino, and ArduinoConnected is a flag specifying if the
% connection was made (1) or not (0).
%
% Version: F. Mino 2023/07/20  

fprintf('%s\n', repmat('-', 1, 80))
disp("Attempting Connection with Arduino")
try
    if nargin 
        board = serial_pkg.SerialComm(varargin); % pass COM port string if specified
    else
        board = serial_pkg.SerialComm();
    end
    connected = true;
    fprintf("\tArduino connected! :)\n")
catch
    board = NaN;
    connected = false;
    fprintf("\tArduino failed to connect! :(\n")
end
end