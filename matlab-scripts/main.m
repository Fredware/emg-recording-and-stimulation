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

%% Section 0A: Clear the workspace
% This line deletes any information from previous sessions.
close all; clear all; clc

%% Section 0B: Run MuJoCo
% The following line will initialize MuJoCo. Once the program opens, load the
% appropriate model and click the "Run" button in MuJoCo.
system("..\mujoco\program\mjhaptix.exe &")

%% Section 1: Setting up MuJoCo
[joint_positions, joint_groups, mujoco_command, mujoco_connected] = mujoco_pkg.connect_hand();

%% Section 2: Setting up Backyard Brains SpikerShield
try arduino_uno.close; catch; end
[arduino_uno, uno_connected] = arduino_pkg.connect_board();
stimbox_connected = 0;

%% Section 3: Setting up the Stim Box
try
    stimbox_connected = stim_pkg.set_stimbox('init');
catch
    stimbox_connected = false;
end

%% PROGRAM SETUP
CTRL_GAIN = 5; % control signal amplification
CTRL_THRESH = 0; % 0: proportional control; (0,1]: binary control
CTRL_WIN_LEN = 300; % samples
STIM_AMPLITUDE = 2.5;
STIM_FREQ = 0;
STIM_ENCODING = 'Linear'; % Encoding scheme; Options: ['Fixed', 'RA1', 'SA1', 'Biomimetic', 'Linear']
MUJOCO_SENSOR_SELECTION = 0; % 0: Uses the most active sensor

%% Section 4: Recording, controlling and stimulating in real-time
fs = arduino_uno.SAMPLING_FREQ;
n_rec_chans = arduino_uno.N_CHANS;
n_ctrl_chans = n_rec_chans;
hand_sensors = zeros(19,6);

% Set up plot
main_fig = rtplot_pkg.initialize_figure(n_rec_chans, n_ctrl_chans, CTRL_THRESH, stimbox_connected);

%Set up data buffers
[emg_buffer, ctrl_buffer, force_buffer, freq_buffer] = initialize_data_structures(n_rec_chans, n_ctrl_chans);
%%
while (ishandle(main_fig.handle))
    pause(0.0111)
    % GET DATA FROM BYB
    try
        emg = arduino_uno.get_recent_emg;
        if ~isempty(emg)
            new_sample_count = size(emg,1);
            emg_buffer.data(emg_buffer.ptr:emg_buffer.ptr+new_sample_count-1,:) = emg;
            emg_buffer.ptr = emg_buffer.ptr + new_sample_count;
            ctrl_buffer.ptr = ctrl_buffer.ptr + 1;
            force_buffer.ptr = force_buffer.ptr + 1;
            freq_buffer.ptr = freq_buffer.ptr + 1;
        end
    catch
        disp("Data acquisition failed")
    end

    if ~isempty(emg) && (emg_buffer.ptr > CTRL_WIN_LEN)
        timestamp = (emg_buffer.ptr - 1)/fs;

        % CALCULATE CONTROL VALUE
        try
            ctrl_emg = emg_buffer.data(emg_buffer.ptr-CTRL_WIN_LEN:emg_buffer.ptr-1,:);
            ctrl_value = CTRL_GAIN * mean(abs(ctrl_emg - mean(ctrl_emg)));
            if(CTRL_THRESH>0)
                ctrl_value = ctrl_value > CTRL_THRESH;
            end
            ctrl_buffer.data(ctrl_buffer.ptr, :) = ctrl_value;

            force_buffer.data(force_buffer.ptr, :) = max(hand_sensors(:,1));
            freq_buffer.data(freq_buffer.ptr, :) = STIM_FREQ;
        catch
            disp("Control calculation failed")
        end
        ctrl_buffer.timestamps(ctrl_buffer.ptr) = timestamp;
        force_buffer.timestamps(force_buffer.ptr) = timestamp;
        freq_buffer.timestamps(freq_buffer.ptr) = timestamp;

        % UPDATE PLOT
        main_fig = rtplot_pkg.update_figure(main_fig, timestamp, emg_buffer, ctrl_buffer, force_buffer, freq_buffer, CTRL_THRESH, stimbox_connected);
        emg_buffer.ptr_prev = emg_buffer.ptr;

        % UPDATE HAND
        if mujoco_connected
            % MOVE HAND
            mujoco_status = mujoco_pkg.update_hand(ctrl_buffer, mujoco_command, joint_groups);
            % READ SENSORS
            mujoco_sensors = circshift(mujoco_sensors, [0,1]);
            mujoco_sensors(:,1) = mujoco_status.contact;
            % STIMULATE
            if stimbox_connected
                stim_freq = stim_pkg.calculate_freq(mujoco_sensors, MUJOCO_SENSOR_SELECTION, STIM_ENCODING);
                stim_pkg.set_stimbox('stim', [min(STIM_AMPLITUDE, 5), 200, stim_freq, 1])
            end
        end
    end
end
arduino_uno.close;
if(stimbox_connected)
    ctrlArduinoStim_hrc('stim');
end
