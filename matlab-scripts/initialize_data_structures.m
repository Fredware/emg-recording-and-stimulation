function [emg_buffer, ctrl_buffer, force_buffer, freq_buffer] = initialize_data_structures(n_rec_chans, n_ctrl_chans)
buffer_len = 600*1e3; %60 sec @ 1kHz

emg_buffer.data = NaN(buffer_len, n_rec_chans);
emg_buffer.ptr = 1;
emg_buffer.ptr_prev = 1;
emg_buffer.timestamps = [0];

ctrl_buffer.data = NaN(buffer_len, n_ctrl_chans);
ctrl_buffer.ptr = 0;
ctrl_buffer.timestamps = [];

force_buffer.data = NaN(buffer_len, 1);
force_buffer.ptr = 0;
force_buffer.timestamps = [];

freq_buffer.data = NaN(buffer_len, 1);
freq_buffer.ptr = 0;
freq_buffer.timestamps = [];
end