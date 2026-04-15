function bms_gui_sim
% ============================================================
% EV BMS GUI SIMULATOR — FIXED TextArea Value bug
% ============================================================
clear; clc;

%% ---------------- GUI SETUP ----------------
fig = uifigure('Name','EV BMS Simulator','Position',[200 200 1100 650]);

% Panels
ctrlPanel = uipanel(fig,'Title','Controls','Position',[20 20 300 610]);
plotPanel = uipanel(fig,'Title','Live Graphs','Position',[340 20 740 610]);

%% --- Controls ---
uicontrol(ctrlPanel,'style','text','String','Throttle Mode','Position',[20 550 120 30],'FontSize',12);
modeSwitch = uiswitch(ctrlPanel,'toggle','Items',{'Auto','Manual'}, ...
    'Position',[150 550 60 40]);

uicontrol(ctrlPanel,'style','text','String','Manual Throttle (%)','Position',[20 500 150 20]);
thrSlider = uislider(ctrlPanel,'Position',[20 480 260 3],'Limits',[0 100],'Value',20);

uicontrol(ctrlPanel,'style','text','String','Runtime (s)','Position',[20 430 120 20]);
runtimeBox = uieditfield(ctrlPanel,'numeric','Position',[150 430 80 22],'Value',300);

startBtn = uibutton(ctrlPanel,'push','Text','Start','Position',[20 380 100 40]);
stopBtn  = uibutton(ctrlPanel,'push','Text','Stop','Position',[140 380 100 40],'Enable','off');

% Quick readings
voltageLabel = uilabel(ctrlPanel,'Position',[20 330 260 20],'Text','Voltage: -- V');
currentLabel = uilabel(ctrlPanel,'Position',[20 305 260 20],'Text','Current: -- A');
socLabel     = uilabel(ctrlPanel,'Position',[20 280 260 20],'Text','SOC: -- %');
tempLabel    = uilabel(ctrlPanel,'Position',[20 255 260 20],'Text','Temp: -- °C');

ledGreen = uilabel(ctrlPanel,'Position',[20 220 260 20],'Text','LED Green: OFF');
ledRed   = uilabel(ctrlPanel,'Position',[20 195 260 20],'Text','LED Red: OFF');
buzz     = uilabel(ctrlPanel,'Position',[20 170 260 20],'Text','Buzzer: OFF');
contLB   = uilabel(ctrlPanel,'Position',[20 145 260 20],'Text','Contactor: OPEN');

% CAN log window
uilabel(ctrlPanel,'Text','CAN Output','Position',[20 120 200 20]);
canBox = uitextarea(ctrlPanel,'Position',[20 20 260 100],'Editable','off');

%% --- Plot axes ---
ax1 = uiaxes(plotPanel,'Position',[20 330 700 250]); title(ax1,'Pack Voltage (V)');
ax2 = uiaxes(plotPanel,'Position',[20 70 700 250]); title(ax2,'Pack Current (A)');

% Plot line handles (set after simulation starts)
voltageLine = [];
currentLine = [];

%% ---------------- SIMULATION PARAMETERS ----------------
params = load_default_params();

%% ---------------- TIMER SETUP ----------------
simTimer = timer('ExecutionMode','fixedRate','Period',params.dt, ...
    'TimerFcn',@(~,~) sim_step(),'BusyMode','drop','ErrorFcn',@timerError);

running = false;

%% ============================================================
% ---------------- CALLBACK: START -----------------------------
%% ============================================================
startBtn.ButtonPushedFcn = @(~,~) start_sim();

    function start_sim()
        if running, return; end
        running = true;
        
        % UI states
        stopBtn.Enable = 'on';
        startBtn.Enable = 'off';
        % Initialize CAN box as an N-by-1 cell array (empty)
        canBox.Value = cell(0,1);
        
        % Reset simulation state
        S = init_state(params, runtimeBox.Value);
        
        % Create plot lines
        voltageLine = plot(ax1, S.timePlot, zeros(size(S.timePlot)), 'LineWidth',1.4);
        currentLine = plot(ax2, S.timePlot, zeros(size(S.timePlot)), 'LineWidth',1.4);
        
        ax1.YLim = [30 42];
        ax2.YLim = [-10 150];
        
        simTimer.UserData = struct('S',S, 'params',params, 'canBox', canBox);
        
        start(simTimer);
    end

%% ============================================================
% ---------------- CALLBACK: STOP ------------------------------
%% ============================================================
stopBtn.ButtonPushedFcn = @(~,~) stop_sim();

    function stop_sim()
        if ~running, return; end
        running = false;
        
        stop(simTimer);
        
        startBtn.Enable = 'on';
        stopBtn.Enable = 'off';
    end

%% ============================================================
% ---------------- SIMULATION STEP -----------------------------
%% ============================================================
    function sim_step()
        D = simTimer.UserData;
        S = D.S;  params = D.params;
        tnow = S.t;
        
        % Stop condition
        if S.t >= S.runtime
            stop_sim();
            return;
        end
        
        S.t = S.t + params.dt;
        
        % ---------------- throttle ----------------
        if strcmp(modeSwitch.Value,'Manual')
            thr = thrSlider.Value / 100;
        else
            thr = throttle_auto(S.t);
        end
        S.throttle = thr;
        
        % ---------------- battery model ----------------
        OCV = pack_ocv(params, S.SOC);
        
        if S.contactor_closed
            I_motor = thr * params.I_nom_each * 2;   % 2 motors
        else
            I_motor = 0;
        end
        
        if S.precharge
            I_pack = OCV / params.R_pre;
        else
            I_pack = I_motor;
        end
        
        Vterm = OCV - I_pack * params.Rint_pack;
        
        % SOC update
        S.SOC = max(0, S.SOC - (I_pack * params.dt)/params.C_As);
        
        % Temperature
        heatW = (I_pack^2) * params.Rint_pack / 3;
        for ci=1:3
            S.Tcell(ci) = S.Tcell(ci) + params.dt*(heatW - params.h_loss*(S.Tcell(ci)-params.Tamb))/params.Cth;
        end
        
        % Precharge → close contactor (simple)
        if S.t > 1 && S.precharge
            S.precharge = false;
            S.contactor_closed = true;
        end
        
        % Protections
        if Vterm < params.V_cutoff
            S.fault = 1;
        elseif I_pack > params.I_instant_trip
            S.fault = 2;
        elseif max(S.Tcell) > params.T_trip
            S.fault = 3;
        end
        
        if S.fault ~= 0
            S.contactor_closed = false;
        end
        
        % Update plots
        S.voltHist = [S.voltHist(2:end) Vterm];
        S.currHist = [S.currHist(2:end) I_pack];
        
        voltageLine.YData = S.voltHist;
        currentLine.YData = S.currHist;
        
        % Update numeric labels
        voltageLabel.Text = sprintf('Voltage: %.2f V', Vterm);
        currentLabel.Text = sprintf('Current: %.1f A', I_pack);
        socLabel.Text = sprintf('SOC: %.1f %%', S.SOC*100);
        tempLabel.Text = sprintf('Temp: %.1f °C', max(S.Tcell));
        
        % Indicators
        ledGreen.Text = sprintf('LED Green: %s', onoff(S.contactor_closed && S.fault==0));
        ledRed.Text   = sprintf('LED Red: %s', onoff(S.fault~=0));
        buzz.Text     = sprintf('Buzzer: %s', onoff(S.fault~=0));
        contLB.Text   = sprintf('Contactor: %s', onoff(S.contactor_closed));
        
        % CAN message simulation (pack basic fields)
        msg = sprintf('t=%.1f V=%.2f I=%.2f SOC=%.1f F=%d', S.t, Vterm, I_pack, S.SOC*100, S.fault);
        % ensure existing canBox.Value is a column cell array of char vectors
        curVal = canBox.Value;
        if isstring(curVal) || ischar(curVal)
            curCell = cellstr(curVal);
        else
            curCell = curVal;
        end
        % Prepend newest message, keep at most 200 lines
        newCell = [{msg}; curCell(:)];
        if numel(newCell) > 200, newCell = newCell(1:200); end
        canBox.Value = newCell;
        
        % Save state
        D.S = S;
        simTimer.UserData = D;
    end

%% ============================================================
% ---------------- TIMER ERROR HANDLER ------------------------
%% ============================================================
    function timerError(~,evt)
        disp('Timer error:');
        disp(evt);
    end

%% ============================================================
% ---------------- HELPER FUNCTIONS ----------------------------
%% ============================================================

function params = load_default_params()
    params.dt = 0.2;
    params.runtime = 300;
    params.Ncells = 3;
    
    params.C_Ah = 7.2;
    params.C_As = params.C_Ah*3600;
    
    params.Rint_cell = 0.03;
    params.Rint_pack = params.Rint_cell * params.Ncells;
    
    params.OCV_full_cell = 12.6;
    params.OCV_empty_cell = 11.8;
    
    params.P_motor_nom = 350;
    params.V_nom = 36;
    params.I_nom_each = params.P_motor_nom/params.V_nom;
    
    params.R_pre = 68;
    
    params.V_cutoff = 35.0;
    params.I_instant_trip = 120;
    params.T_trip = 55;
    
    params.h_loss = 1.2;
    params.Tamb = 25;
    params.Cth = 900;
end

function S = init_state(params, runtime)
    S.t = 0;
    S.SOC = 1.0;
    S.Tcell = [25 25 25];
    
    S.fault = 0;
    S.contactor_closed = false;
    S.precharge = true;
    
    S.runtime = runtime;
    
    Np = 200;
    S.timePlot = linspace(-params.dt*Np, 0, Np);
    S.voltHist = zeros(1,Np);
    S.currHist = zeros(1,Np);
end

function thr = throttle_auto(t)
    % simple auto throttle
    thr = 0.3 + 0.2*sin(0.05*t) + 0.1*(randn*0.05);
    thr = max(0,min(1,thr));
end

function v = pack_ocv(p, soc)
    vcell = p.OCV_empty_cell + (p.OCV_full_cell - p.OCV_empty_cell)*soc;
    v = p.Ncells * vcell;
end

function out = onoff(x)
    if x, out = 'ON'; else, out='OFF'; end
end

end
