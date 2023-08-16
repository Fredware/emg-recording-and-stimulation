function fig = initialize_figure(n_rec_chans, n_ctrl_chans, ctrl_tresh, stim_connected)
PLT_THEME.COLORS = ["#FF1F5B", "#00CD6C", "#009ADE", "#AF58BA", "#FFC61E", "#F28522"];
n_opt_plots = 1;% +1 for mujoco sensors;
if stim_connected
    n_opt_plots = n_opt_plots + 1; % +1 for stim freq
end

n_plots = 2 + n_opt_plots; % rec + ctrl + opt

fig.t_min = 0;
fig.t_max = 5;

fig.n_rec = n_rec_chans;
fig.n_ctrl = n_ctrl_chans;
fig.n_opt = n_opt_plots;

fig.axes_handles = cell(1, n_plots);
fig.axes = cell(1, n_plots);
fig.line_handles = cell(1, n_rec_chans+n_ctrl_chans+n_opt_plots);
y_labels = cell(1, n_plots);
fig.handle = figure('Units','normalized');
set(fig.handle,'outerposition',[0,0,.5,1])%moveFigure to left half of screen

for i=1:n_plots
    switch(i)
        case 1
            fig.axes{i}.handle = subplot(n_plots, 1, i);
            for j=1:n_rec_chans
                fig.axes{i}.line_handles{j} = animatedline(fig.axes{i}.handle, 'Color',PLT_THEME.COLORS(j));
            end
            ylim(fig.axes{i}.handle, [-3, 3])
            xticks(fig.axes{i}.handle, [])
            xlim(fig.axes{i}.handle,[fig.t_min fig.t_max])
            ylabel(fig.axes{i}.handle, "EMG [V]");
        case 2
            fig.axes{i}.handle = subplot(n_plots, 1, i);
            for j=1:n_ctrl_chans
                fig.axes{i}.line_handles{j} = animatedline(fig.axes{i}.handle, 'Color',PLT_THEME.COLORS(j));
            end
            ylim(fig.axes{i}.handle, [0, 3])
            xlim(fig.axes{i}.handle,[fig.t_min fig.t_max])
            ylabel(fig.axes{i}.handle, "Control [V]");
        case 3
            fig.axes{i}.handle = subplot(n_plots, 1, i);
            fig.axes{i}.line_handles{1} = animatedline(fig.axes{i}.handle);
            ylim(fig.axes{i}.handle, [0, 20])
            xlim(fig.axes{i}.handle,[fig.t_min fig.t_max])
            ylabel(fig.axes{i}.handle, "Force [N]");
        case 4
            fig.axes{i}.handle = subplot(n_plots, 1, i);
            fig.axes{i}.line_handles{1} = animatedline(fig.axes{i}.handle);
            ylim(fig.axes{i}.handle, [0, 200])
            xlim(fig.axes{i}.handle,[fig.t_min fig.t_max])
            ylabel(fig.axes{i}.handle, "Frequency [Hz]");
        otherwise
            error("Axis not found")
    end
end

fig_axes = [];
for i=1:n_plots
fig_axes = [fig_axes, fig.axes{i}.handle];
end
linkaxes(fig_axes, 'x')

% for i=1:n_rec_chans
%     y_labels{i} = sprintf("EMG CH %02d [V]", i);
%     fig.axes_handles{i} = subplot(n_plots, 1, i);
%     fig.line_handles{i} = animatedline;
%     xlim([fig.t_min fig.t_max])
%     xticks([])
%     ylim([-2.5 2.5])
%     ylabel(y_labels{i})
% end
% 
% for i=1:n_ctrl_chans
%     y_labels{n_rec_chans+i} = sprintf("CTRL CH %02d", i);
%     fig.axes_handles{n_rec_chans+i} = subplot(n_plots, 1, n_rec_chans+i);
%     fig.line_handles{n_rec_chans+i} = animatedline;
%     ylim([-0.25,1.25]);
%     yticks([0,1]);
%     yticklabels({'OPEN (0)','CLOSE (1)'})
%     if(ctrl_tresh > 0)
%         hold on;
%         plot([0,Tmax],[THRESH,THRESH],'r')
%     end
%     ylabel(y_labels(n_rec_chans+i))
% end
% 
% for i=1:n_opt_plots
%     if i==1 && stim_connected
%         y_labels{n_rec_chans+n_ctrl_chans+i} = sprintf("STIM FREQ");
%         fig.axes_handles{n_rec_chans+n_ctrl_chans+i} = subplot(n_plots, 1, n_rec_chans + n_ctrl_chans+i);
%         fig.line_handles{n_rec_chans+n_ctrl_chans+i} = animatedline;
%         ylim([0,20]);
% 
%     else
%         y_labels{n_rec_chans+n_ctrl_chans+i} = sprintf("FORCE [N]");
%         fig.axes_handles{n_rec_chans+n_ctrl_chans+i} = subplot(n_plots, 1, n_rec_chans+ n_ctrl_chans+i);
%         fig.line_handles{n_rec_chans+n_ctrl_chans+i} = animatedline;
%         ylim([0,200]);
% 
%     end
%     ylabel(y_labels(n_rec_chans + n_ctrl_chans+i))
% end
% 
% linkaxes([fig.axes_handles{:}],'x')
% xlabel('Time (seconds)')
end