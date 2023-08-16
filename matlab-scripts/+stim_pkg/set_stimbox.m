function [runflag, varargout] = set_stimbox(varargin)
% Amp is in mA. Freq is in Hz. Run "init" mode first.
% Arduino sketch:
% "\Box\NeuroroboticsLab\JAGLAB\Projects\StimShield\Software\HVStimShieldMultiChannel\MultiChannel_StimShield_mat20210607\MultiChannel_StimShield_mat20210607.ino"
%
% ctrlHowlandStim or ctrlHowlandStim('init')
% ctrlHowlandStim('stim',[1,100,200,0,1,100,200,0,1,100,200,0]) %stim amp1 (1mA), pulse freq1 (5Hz),
% pulsewidth1 (us), offset1 (ms), amp2 (1mA)...
% ctrlHowlandStim('close')
%
% Version: 20210607
% Author: Tyler Davis, Marshall Trout
%
% Version: 20230724
% Author Fredi Mino

persistent ARD
persistent COMStr

mode = [];
ard_time = -1;

if nargin
    for k=1:length(varargin)
        if strcmp(varargin{k},'init')
            if isempty(ARD)
                ard_time = 0;
                mode = 'init';
                if length(varargin)>k
                    com_port = varargin{k+1};
                else
                    com_port = [];
                end
            end
            break;
        elseif strcmp(varargin{k},'stim')
            mode = 'stim';
            ard_time = 0;
            if length(varargin)>k
                stim = varargin{k+1};
                if size(stim,2) == 4
                    stim = [stim(1:2), zeros(1,4), stim(3:4)];
                end
            else
                stim = [0,0,0,0,0,0,0,0];
            end
            break;
        elseif strcmp(varargin{k},'close')
            ard_time = 0;
            mode = 'close';
            break;
        end
    end
else
    if isempty(ARD)
        mode = 'init';
    end
end

try
    if ~isempty(mode)

        switch mode
            case 'init'
                if isempty(com_port)
                    devs = serial_pkg.get_serial_id();
                    if ~isempty(devs)
                        com_port = cell2mat(devs(~cellfun(@isempty,regexp(devs(:,1),'Arduino Uno')),2));
                        if isempty(com_port)
                            com_port = cell2mat(devs(~cellfun(@isempty,regexp(devs(:,1),'USB-SERIAL CH340')),2));
                        end
                    end
                end
                if ~isempty(com_port)
                    COMStr = sprintf('COM%0.0f',com_port(end));
                end
                delete(instrfind('port',COMStr));
                ARD = serial(COMStr,'BaudRate',115200,'Timeout',0.033);
                fopen(ARD); pause(5);
                flushinput(ARD); pause(0.1);
                flushoutput(ARD); pause(0.1);
                fwrite(ARD,[0,0,0,0,0,0,0,0],'uint8'); pause(0.1)
                val = fread(ARD,4);
                ard_time = val(1)*2^0+val(2)*2^8+val(3)*2^16+val(4)*2^24;
                disp('StimBox connected...');
                %disp(ard_time);
                %                 varargout{1} = ard_time;

            case 'stim'

                potval1 = floor(stim(1)*16.25);
                potval1(potval1<0) = 0;
                potval1(potval1>255) = 255;

                %                 freq1 = floor(stim(2));
                %                 freq1(freq1<0) = 0;
                %                 freq1(freq1>255) = 255;

                pulsewidth1 = floor(stim(2)*.2);
                pulsewidth1(pulsewidth1<0) = 0;
                pulsewidth1(pulsewidth1>255) = 255;

                %                 offset1 = max(floor(stim(4)), 0);
                %                 offset1 (offset1>255) = 255;



                potval2 = floor(stim(3)*26.25);
                potval2(potval2<0) = 0;
                potval2(potval2>255) = 255;

                %                 freq2 = floor(stim(6));
                %                 freq2(freq2<0) = 0;
                %                 freq2(freq2>255) = 255;

                pulsewidth2 = floor(stim(4)*.2);
                pulsewidth2(pulsewidth2<0) = 0;
                pulsewidth2(pulsewidth2>255) = 255;

                %                 offset2 = max(floor(stim(8)), 0);
                %                 offset2 (offset2>255) = 255;




                potval3 = floor(stim(5)*16.25);
                potval3(potval3<0) = 0;
                potval3(potval3>255) = 255;

                %                 freq3 = floor(stim(10));
                %                 freq3(freq3<0) = 0;
                %                 freq3(freq3>255) = 255;

                pulsewidth3 = floor(stim(6)*.2);
                pulsewidth3(pulsewidth3<0) = 0;
                pulsewidth3(pulsewidth3>255) = 255;

                %                 offset3 = max(floor(stim(12)), 0);
                %                 offset3 (offset3>255) = 255;


                freq = floor(stim(7));
                freq(freq<0) = 0;
                freq(freq>255) = 255;

                if stim(8) == 1
                    interleaved = stim(8);
                else
                    interleaved = 0;
                end

                %                 maxfs = floor(1000/freq1-1); %max fast settle width in msec (pulse interval minus stim pulse width (0.5ms))
                %                 maxfs(maxfs>50) = 50;
                %                 fs = floor(stim(4));
                %                 fs(fs<0) = 0;
                %                 fs(fs>maxfs) = maxfs; %max 50ms for fast settle (this doubles as sync pulse)

                if isobject(ARD)
                    fwrite(ARD,[potval1,pulsewidth1,...
                        potval2,pulsewidth2,...
                        potval3,pulsewidth3,...
                        freq, interleaved],'uint8'); %amp, freq, pw, offset
                    val = fread(ARD,4);
                    ard_time = val(1)*2^0+val(2)*2^8+val(3)*2^16+val(4)*2^24;
                else
                    disp('Connection Err...');
                    ard_time = -1;
                end

            case 'close'
                if isobject(ARD)
                    fwrite(ARD,[0,0,0,0,0,0,0,0,0,0,0,0],'uint8');
                    val = fread(ARD,4);
                    ard_time = val(1)*2^0+val(2)*2^8+val(3)*2^16+val(4)*2^24;
                    fclose(ARD);
                    delete(ARD);
                end
                ARD = [];
                COMStr = [];

        end

    end
catch ME
    if isobject(ARD)
        fclose(ARD);
        delete(ARD);
    end
    ARD = [];
    COMStr = [];
    fprintf('ctrlHowlandStim: %s; name: %s; line: %0.0f\r\n',ME.message,ME.stack(1).name,ME.stack(1).line);
end

varargout{1} = ard_time;

runflag = false;
if isobject(ARD)
    runflag = true;
end
end