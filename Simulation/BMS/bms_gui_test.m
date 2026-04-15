function bms_gui_test
% Minimal GUI test: opens a uifigure, shows a Start button and a ticking label.
% Save as bms_gui_test.m and run in MATLAB: bms_gui_test

% simple cleanup
clear classes; close all;

% create UI
fig = uifigure('Name','BMS GUI Test','Position',[300 300 400 200]);
lbl = uilabel(fig,'Position',[20 120 360 30],'Text','Press Start to run timer','FontSize',14);
btn = uibutton(fig,'push','Text','Start','Position',[20 60 100 40]);
btn2 = uibutton(fig,'push','Text','Stop','Position',[140 60 100 40],'Enable','off');

% ticking label
tickLabel = uilabel(fig,'Position',[20 20 360 30],'Text','t = 0.0 s','FontSize',12);

% timer setup
t = timer('ExecutionMode','fixedRate','Period',0.2,'TimerFcn',@(~,~) updateTick());

running = false;
tcount = 0;

    function startCB(~,~)
        if running, return; end
        running = true;
        btn.Enable = 'off';
        btn2.Enable = 'on';
        start(t);
    end

    function stopCB(~,~)
        if ~running, return; end
        running = false;
        btn.Enable = 'on';
        btn2.Enable = 'off';
        stop(t);
    end

    function updateTick()
        tcount = tcount + 0.2;
        % update UI on main thread
        if isvalid(fig)
            tickLabel.Text = sprintf('t = %.1f s', tcount);
        else
            stop(t);
            delete(t);
        end
    end

btn.ButtonPushedFcn = @startCB;
btn2.ButtonPushedFcn = @stopCB;

end
