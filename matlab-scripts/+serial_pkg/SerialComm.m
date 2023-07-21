classdef SerialComm < handle
    % This class for connecting to, reading from, and closing the TASKA fingertip sensors
    %
    % Note: It is currently hard-coded for eight sensors (4 IR, 4 baro),
    % but could be adapted for other sensor counts
    %
    % Example usage:
    % TS = TASKASensors;
    % TS.Status.IR or TS.Status.BARO would display IR or baro data,
    % respectively
    %
    % Version: 20210222
    % Author: Tyler Davis
    %
    % Version: 20230710
    % Author: Fredi Mino
    
    properties
        arduino; 
        port_number;
        sample_count; 
        data_buffer;
        status;
        is_ready;

        BAUD_RATE = 250000;
        MESSAGE_FORMAT = "%d %d %d %d %d %d";
    end
    
    methods
        function obj = SerialComm( varargin)
            % Optional args:            
            % {1}: Number of channels
            % {2}: COM port number

            obj.is_ready = false;
            
            if nargin > 0
                n_chans = varargin{1};
            else
                n_chans = 6;
            end

            % Buffer size = [(2 sec @ 1 kHz = 1,500 samples), (num_chans)]
            n_samples = 2000;
            
            obj.data_buffer = zeros(n_samples, n_chans);
            
            obj.status.elapsed_time = nan;
            obj.status.current_time = clock;
            obj.status.prior_time = clock;
            obj.sample_count = 0;
            
            if nargin > 1
                COMPort = varargin{2};
                init( obj, COMPort);
            else
                init( obj);
            end
        end
        
        function init( obj, varargin)
            if nargin > 1
                COMPort = varargin{1};
                if ~isempty( COMPort)
                    obj.port_number = sprintf( 'COM%0.0f', COMPort(1));
                end
            else
                devs = serial_pkg.get_serial_id;
                if ~isempty(devs)
                    COMPort = cell2mat(devs(~cellfun(@isempty,regexp(devs(:,1),'Arduino Uno')),2));
                    if ~isempty(COMPort)
                        obj.port_number = sprintf('COM%0.0f',COMPort(1));
                    else
                        COMPort = cell2mat(devs(~cellfun(@isempty,regexp(devs(:,1),'USB-SERIAL CH340')),2));
                        if ~isempty(COMPort)
                            obj.port_number = sprintf('COM%0.0f',COMPort(1));
                        end
                    end
                end
            end
            delete(instrfind('port',obj.port_number));
            obj.arduino = serialport(obj.port_number, obj.BAUD_RATE, 'Timeout', 1);
            configureCallback(obj.arduino,"terminator",@obj.read_serial);
            flush(obj.arduino);
            pause(0.1);
            obj.is_ready = true;
        end
        
        function close( obj, varargin)
            if isobject( obj.arduino)
                delete( obj.arduino);
            end
        end
        
        function read_serial( obj, varargin)
            try
                % read data & update status
                obj.status.data = sscanf( readline( obj.arduino), obj.MESSAGE_FORMAT);
                obj.status.current_time = clock;
                obj.status.elapsed_time = etime( obj.status.current_time, obj.status.prior_time);
                obj.status.prior_time = obj.status.current_time;
                
                % store data into buffer
                obj.data_buffer = circshift( obj.data_buffer, [-1 0]);
                obj.data_buffer( end, :) = obj.status.data;
                
                obj.sample_count = obj.sample_count + 1;
            catch
                disp('Serial communication error!')
            end
        end
        
        function emg = get_emg( obj, varargin)
            % - divide by highest value (1024) to normalize signal
            % - multiply by 5 to match the signal recorded by the SpikerShield (0V-5V)
            % - subtract 2.5 V to make the signal zero-centered and undo the
            %   shift introduced by BYB  
            emg = obj.data_buffer / 1024 * 5 - 2.5;
        end
        
        function emg = get_recent_emg( obj, varargin)
            last_sample_idx = size( obj.data_buffer, 1); % return the rows the buffer
            first_sample_idx = last_sample_idx - obj.sample_count;
            if first_sample_idx < 1
                first_sample_idx = 1;
            end
            if first_sample_idx == last_sample_idx
                emg = [];
            else
                emg = obj.data_buffer(first_sample_idx:end,:) / 1024 * 5 - 2.5;
                obj.sample_count = 0;
            end
%             varargout{1} = obj.Status.current_time;
%             varargout{2} = length(EMG) / ( obj.Status.current_time - obj.Status.prior_time);
        end
    end    
end %class
