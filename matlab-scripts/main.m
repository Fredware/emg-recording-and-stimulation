%% COPYRIGHT NOTICE
%     This program implements the recording and stimulation functionality
%     of the "Jedi Scientist" demo.
%     Copyright (C) 2023  Fredi R. Mino
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <https://www.gnu.org/licenses/>.

%% INSTRUCTIONS
% 1) Ensure MuJoCo is open and running before proceeding with the script.
% 2) The script is divided into three sections. You have the option to run
% the entire script or each section individually.
%
% Section 1: Setting up MuJoCo
%   This section establishes the connection between MATLAB and MuJoCo.
% Section 2: Setting up Backyard Brains Arduino
%   This section establishes the connection between MATLAB and the Backyard
%   Brains System.
% Section 3: Setting up Stim Box
%   This section establishes the connection between MATLAB and the Stim Box
% Section 4: Recording and Controlling in Real-time.
%   This section continuously acquires signals from the Backyard Brains
%   system, plots the signals, and uses them to control the hand in MuJoCo.

%% Section 0: Prepare the workspace
% This line deletes any information from previous sessions.
clearvars
% The following line will initialize MuJoCo. Once the program opens, load the
% appropriate model and click the "Run" button in MuJoCo.
system("..\mujoco\program\mjhaptix.exe &")

%% Section 1: Setting up MuJoCo
[joint_positions, joint_groups, command, mujoco_connected] = mujoco_pkg.connect_hand();

%% Section 2: Setting up Backyard Brains SpikerShield
try uno.close; catch; end
[uno, uno_connected] = arduino_pkg.connect_board();
