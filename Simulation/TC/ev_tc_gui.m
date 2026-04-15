function ev_tc_gui()
% ev_tc_gui - EV + Traction Control simulator with interactive GUI
% Save this file as ev_tc_gui.m and run in MATLAB.
%
% Features:
% - Manual / Automatic throttle mode
% - Brake slider
% - Traction Control ON/OFF
% - Start / Stop simulation
% - Live plots: RPMs, currents, pack voltage, SoC
% - Numeric readouts for motor rpm & currents
%
% Notes:
% - This is a system-level simplified model (not per-phase BLDC).
% - If you need adjustments (motor constants, battery capacity, dt), edit the params below.

%% Simulation parameters (tweak as needed)
params.Tsim = 600;        % default total sim time (s) for automatic mode (unused for GUI live)
params.dt = 0.02;         % simulation timestep (s)
params.V_nom = 36.0;      % pack nominal voltage [V]
params.Rb = 0.02;         % battery internal resistance [ohm]
params.Ah = 7.2;          % battery capacity [Ah]
params.Q = params.Ah * 3600; % coulombs
params.noload_rpm = 4500; % desired no-load RPM at nominal voltage
params.I_cont_max = 120;  % controller steady current limit [A]
params.I_cont_peak = 250; % used to set R_eq
params.Kt_factor = 1.0;   % multiply Kt baseline
params.load_torque = 0.6; % external torque [N*m]
params.J = 0.02;          % inertia [kg*m^2]
params.b = 0.0008;        % viscous damping
params.SLIP_THRESHOLD = 0.15;   % 15% slip threshold
params.SLIP_DEBOUNCE = 0.12;    % seconds slip must persist
params.RECOVERY_DEBOUNCE = 0.25;% seconds to recover
params.MAX_REDUCTION = 0.6;     % reduce throttle by up to 60% (i.e., to 40%)
params.R_eq_min = 0.003;        % minimum equivalent resistance
params.PWM_RAMP_STEP = 0.02;    % how much throttle changes per step when TC reduces (0..1)

% compute motor constants from noload rpm (SI)
omega0 = params.noload_rpm * 2*pi/60;
params.Ke = params.V_nom / omega0; % V per rad/s
params.Kt = params.Ke * params.Kt_factor; % N*m/A

%% Build GUI
h.fig = figure('Name','EV Traction Control Simulator','NumberTitle','off','Position',[100 100 1200 700]);

% Left panel: controls
uicontrol(h.fig,'Style','text','String','Throttle Mode','FontWeight','bold','Position',[20 640 140 20],'HorizontalAlignment','left');
h.modeGroup = uibuttongroup('Parent',h.fig,'Position',[0.02 0.84 0.16 0.08]);
h.radioManual = uicontrol(h.modeGroup,'Style','radiobutton','String','Manual','Units','normalized','Position',[0.05 0.55 0.9 0.35],'Value',1);
h.radioAuto = uicontrol(h.modeGroup,'Style','radiobutton','String','Automatic','Units','normalized','Position',[0.05 0.05 0.9 0.35],'Value',0);

uicontrol(h.fig,'Style','text','String','Manual Throttle','Position',[20 590 140 18],'HorizontalAlignment','left');
h.sldThrottle = uicontrol(h.fig,'Style','slider','Min',0,'Max',1,'Value',0,'Position',[20 560 200 25],'Callback',@(~,~) updateDisplay());
h.txtThrottleVal = uicontrol(h.fig,'Style','text','String','0.00','Position',[230 560 50 25]);

uicontrol(h.fig,'Style','text','String','Brake (0-1)','Position',[20 520 140 18],'HorizontalAlignment','left');
h.sldBrake = uicontrol(h.fig,'Style','slider','Min',0,'Max',1,'Value',0,'Position',[20 490 200 25],'Callback',@(~,~) updateDisplay());
h.txtBrakeVal = uicontrol(h.fig,'Style','text','String','0.00','Position',[230 490 50 25]);

h.chkTC = uicontrol(h.fig,'Style','checkbox','String','Traction Control ON','Value',1,'Position',[20 450 160 25]);

% Automatic throttle settings (simple profile)
uicontrol(h.fig,'Style','text','String','Auto Profile: step to','Position',[20 420 140 16],'HorizontalAlignment','left');
h.editAutoVal = uicontrol(h.fig,'Style','edit','String','0.8','Position',[150 420 40 22],'Callback',@(~,~) []);
uicontrol(h.fig,'Style','text','String','at t(s)','Position',[200 420 50 16],'HorizontalAlignment','left');
h.editAutoT = uicontrol(h.fig,'Style','edit','String','5','Position',[250 420 40 22],'Callback',@(~,~) []);

% Controls: Start / Stop / Reset
h.btnStart = uicontrol(h.fig,'Style','pushbutton','String','Start','Position',[20 370 80 30],'Callback',@startSim);
h.btnStop = uicontrol(h.fig,'Style','pushbutton','String','Stop','Position',[110 370 80 30],'Callback',@stopSim,'Enable','off');
h.btnReset = uicontrol(h.fig,'Style','pushbutton','String','Reset','Position',[200 370 80 30],'Callback',@resetSim);

% Numeric readouts
uicontrol(h.fig,'Style','text','String','Motor1 RPM:','Position',[20 330 100 18],'HorizontalAlignment','left');
h.lblRPM1 = uicontrol(h.fig,'Style','text','String','0.0','Position',[120 330 80 18]);
uicontrol(h.fig,'Style','text','String','Motor2 RPM:','Position',[20 305 100 18],'HorizontalAlignment','left');
h.lblRPM2 = uicontrol(h.fig,'Style','text','String','0.0','Position',[120 305 80 18]);

uicontrol(h.fig,'Style','text','String','I1 (A):','Position',[20 275 100 18],'HorizontalAlignment','left');
h.lblI1 = uicontrol(h.fig,'Style','text','String','0.0','Position',[120 275 80 18]);
uicontrol(h.fig,'Style','text','String','I2 (A):','Position',[20 250 100 18],'HorizontalAlignment','left');
h.lblI2 = uicontrol(h.fig,'Style','text','String','0.0','Position',[120 250 80 18]);

uicontrol(h.fig,'Style','text','String','Vpack (V):','Position',[20 220 100 18],'HorizontalAlignment','left');
h.lblV = uicontrol(h.fig,'Style','text','String',sprintf('%.2f',params.V_nom),'Position',[120 220 80 18]);
uicontrol(h.fig,'Style','text','String','SoC (%):','Position',[20 195 100 18],'HorizontalAlignment','left');
h.lblSoC = uicontrol(h.fig,'Style','text','String','100.0','Position',[120 195 80 18]);

% Axes: plots
ax1 = axes('Parent',h.fig,'Units','pixels','Position',[360 380 800 300]);
p1 = plot(ax1,0,0,'b',0,0,'r'); hold(ax1,'on'); title(ax1,'Motor RPMs'); legend(ax1,'RPM1','RPM2'); xlabel(ax1,'Time (s)'); ylabel(ax1,'RPM');
ax2 = axes('Parent',h.fig,'Units','pixels','Position',[360 220 390 140]);
p2 = plot(ax2,0,0,'b',0,0,'r'); hold(ax2,'on'); title(ax2,'Controller Currents (A)'); legend(ax2,'I1','I2'); xlabel(ax2,'Time (s)'); ylabel(ax2,'A');
ax3 = axes('Parent',h.fig,'Units','pixels','Position',[770 220 390 140]);
p3 = plot(ax3,0,0,'k'); title(ax3,'Pack Current (A)'); xlabel(ax3,'Time (s)'); ylabel(ax3,'A'); grid(ax3,'on');
ax4 = axes('Parent',h.fig,'Units','pixels','Position',[360 30 800 160]);
p4 = plot(ax4,0,0,'g'); title(ax4,'State of Charge (SoC)'); xlabel(ax4,'Time (s)'); ylabel(ax4,'SoC'); ylim(ax4,[-0.05 1.05]); grid(ax4,'on');

% store in handle
h.ax1 = ax1; h.ax2 = ax2; h.ax3 = ax3; h.ax4 = ax4;
h.p1 = p1; h.p2 = p2; h.p3 = p3; h.p4 = p4;
h.params = params;

% Simulation state
sim.running = false;
sim.t = 0;
sim.idx = 1;
sim.maxSteps = 200000; % safety upper cap
sim.dt = params.dt;

% allocate data arrays for plotting (grow as needed)
buf.t = []; buf.rpm1 = []; buf.rpm2 = []; buf.I1 = []; buf.I2 = []; buf.Ipack = []; buf.Vpack = []; buf.SOC = [];

% Initialize physical states
state.omega1 = 0; state.omega2 = 0; % rad/s
state.I1 = 0; state.I2 = 0;
state.SOC = 1.0;
state.Vpack = params.V_nom;

% Traction control internal state
tc.slipActive1 = false; tc.slipActive2 = false;
tc.slipStart1 = 0; tc.slipStart2 = 0;
tc.recoverStart1 = 0; tc.recoverStart2 = 0;
tc.pwmOut1 = 0; tc.pwmOut2 = 0; % output throttle 0..1 (after TC)
tc.lastUpdateTime = 0;

% connect nested functions to GUI
guidata(h.fig,h);

% helper: update display text
    function updateDisplay()
        set(h.txtThrottleVal,'String',sprintf('%.2f',get(h.sldThrottle,'Value')));
        set(h.txtBrakeVal,'String',sprintf('%.2f',get(h.sldBrake,'Value')));
    end

updateDisplay();

%% Simulation control functions
    function startSim(~,~)
        if sim.running, return; end
        sim.running = true;
        set(h.btnStart,'Enable','off'); set(h.btnStop,'Enable','on');
        % reset buffers
        buf.t = []; buf.rpm1 = []; buf.rpm2 = []; buf.I1 = []; buf.I2 = []; buf.Ipack = []; buf.Vpack = []; buf.SOC = [];
        % reset states
        state.omega1 = 0; state.omega2 = 0; state.I1 = 0; state.I2 = 0;
        state.SOC = 1.0; state.Vpack = params.V_nom;
        tc.slipActive1 = false; tc.slipActive2 = false;
        tc.slipStart1 = 0; tc.slipStart2 = 0; tc.recoverStart1 = 0; tc.recoverStart2 = 0;
        tc.pwmOut1 = 0; tc.pwmOut2 = 0;
        sim.t = 0;
        sim.idx = 1;
        tc.lastUpdateTime = 0;
        runLoop();
    end

    function stopSim(~,~)
        sim.running = false;
        set(h.btnStart,'Enable','on'); set(h.btnStop,'Enable','off');
    end

    function resetSim(~,~)
        stopSim();
        updateDisplay();
        % clear plots
        cla(h.ax1); cla(h.ax2); cla(h.ax3); cla(h.ax4);
        title(h.ax1,'Motor RPMs'); legend(h.ax1,'RPM1','RPM2'); xlabel(h.ax1,'Time (s)');
        title(h.ax2,'Controller Currents (A)'); legend(h.ax2,'I1','I2');
        title(h.ax3,'Pack Current (A)');
        title(h.ax4,'State of Charge (SoC)'); ylim(h.ax4,[-0.05 1.05]);
    end

%% Core simulation loop (runs synchronously - GUI responsive via drawnow)
    function runLoop()
        tic;
        while sim.running
            tnow = sim.t;
            dt = sim.dt;
            % read controls
            isManual = get(h.radioManual,'Value')==1;
            throttle_cmd = 0;
            if isManual
                throttle_cmd = get(h.sldThrottle,'Value');
            else
                % automatic simple step profile: read target and time
                targ = str2double(get(h.editAutoVal,'String'));
                tstep = str2double(get(h.editAutoT,'String'));
                if isnan(targ), targ = 0.8; end
                if isnan(tstep), tstep = 5; end
                if tnow >= tstep
                    throttle_cmd = targ;
                else
                    throttle_cmd = (tnow / max(tstep,1)) * targ;
                end
            end
            brake_cmd = get(h.sldBrake,'Value');
            tc_on = get(h.chkTC,'Value')==1;
            
            % effective throttle after brake: simple subtraction
            thr_req = max(0, throttle_cmd * (1 - brake_cmd));
            
            % --- traction control algorithm ---
            % reference RPM: average or mapping from throttle
            rpm1 = state.omega1 * 60/(2*pi);
            rpm2 = state.omega2 * 60/(2*pi);
            % map throttle->rpmRef linearly (approx)
            n_ref = params.noload_rpm * thr_req; % simple mapping
            rpmRef = (rpm1 + rpm2)/2;
            if rpmRef < 1, rpmRef = max(1,n_ref); end
            
            if tc_on
                % check slip for motor1
                slip1 = (rpm1 - rpmRef)/max(rpmRef,1);
                if slip1 > params.SLIP_THRESHOLD
                    if ~tc.slipActive1
                        if tc.slipStart1==0, tc.slipStart1 = tnow; end
                        if (tnow - tc.slipStart1) >= params.SLIP_DEBOUNCE
                            tc.slipActive1 = true;
                        end
                    end
                else
                    tc.slipStart1 = 0;
                    if tc.slipActive1
                        if slip1 < (params.SLIP_THRESHOLD*0.3)
                            if tc.recoverStart1==0, tc.recoverStart1 = tnow; end
                            if (tnow - tc.recoverStart1) >= params.RECOVERY_DEBOUNCE
                                tc.slipActive1 = false; tc.recoverStart1 = 0;
                            end
                        else
                            tc.recoverStart1 = 0;
                        end
                    end
                end
                % motor2
                slip2 = (rpm2 - rpmRef)/max(rpmRef,1);
                if slip2 > params.SLIP_THRESHOLD
                    if ~tc.slipActive2
                        if tc.slipStart2==0, tc.slipStart2 = tnow; end
                        if (tnow - tc.slipStart2) >= params.SLIP_DEBOUNCE
                            tc.slipActive2 = true;
                        end
                    end
                else
                    tc.slipStart2 = 0;
                    if tc.slipActive2
                        if slip2 < (params.SLIP_THRESHOLD*0.3)
                            if tc.recoverStart2==0, tc.recoverStart2 = tnow; end
                            if (tnow - tc.recoverStart2) >= params.RECOVERY_DEBOUNCE
                                tc.slipActive2 = false; tc.recoverStart2 = 0;
                            end
                        else
                            tc.recoverStart2 = 0;
                        end
                    end
                end
            else
                % if TC disabled, reset TC flags and use requested throttle directly
                tc.slipActive1 = false; tc.slipActive2 = false;
                tc.slipStart1 = 0; tc.slipStart2 = 0; tc.recoverStart1 = 0; tc.recoverStart2 = 0;
            end
            
            % Determine per-motor commanded throttle after TC
            % Start from thr_req and reduce per-slip flags smoothly
            % Ramp current tc.pwmOut toward new target
            target1 = thr_req;
            target2 = thr_req;
            if tc.slipActive1
                target1 = thr_req * (1 - params.MAX_REDUCTION);
            end
            if tc.slipActive2
                target2 = thr_req * (1 - params.MAX_REDUCTION);
            end
            % ramping
            tc.pwmOut1 = tc.pwmOut1 + sign(target1 - tc.pwmOut1) * min(abs(target1 - tc.pwmOut1), params.PWM_RAMP_STEP);
            tc.pwmOut2 = tc.pwmOut2 + sign(target2 - tc.pwmOut2) * min(abs(target2 - tc.pwmOut2), params.PWM_RAMP_STEP);
            % ensure bounds
            tc.pwmOut1 = max(0,min(1,tc.pwmOut1));
            tc.pwmOut2 = max(0,min(1,tc.pwmOut2));
            
            % --- motor & battery physics (Euler integration) ---
            % Use simplified controller model: Vdrive = Vpack * pwm; current ~ (Vdrive - Vbemf)/R_eq, clamped by I_cont_max
            R_eq = max(params.R_eq_min, params.V_nom / params.I_cont_peak);
            Vbemf1 = params.Ke * state.omega1;
            Vbemf2 = params.Ke * state.omega2;
            Vdrive1 = state.Vpack * tc.pwmOut1;
            Vdrive2 = state.Vpack * tc.pwmOut2;
            I1 = max(0, min(params.I_cont_max, (Vdrive1 - Vbemf1)/R_eq));
            I2 = max(0, min(params.I_cont_max, (Vdrive2 - Vbemf2)/R_eq));
            
            % mechanical torque and acceleration
            Tm1 = params.Kt * I1;
            Tm2 = params.Kt * I2;
            domega1 = (Tm1 - params.load_torque - params.b * state.omega1) / params.J;
            domega2 = (Tm2 - params.load_torque - params.b * state.omega2) / params.J;
            state.omega1 = max(0, state.omega1 + domega1 * dt);
            state.omega2 = max(0, state.omega2 + domega2 * dt);
            
            % battery behavior: pack current and sag and SoC
            Ipack = I1 + I2;
            state.Vpack = params.V_nom - Ipack * params.Rb;
            dQ = Ipack * dt; % coulombs
            state.SOC = max(0, state.SOC - dQ / params.Q);
            
            % store state values for plotting
            buf.t(end+1) = tnow;
            buf.rpm1(end+1) = state.omega1*60/(2*pi);
            buf.rpm2(end+1) = state.omega2*60/(2*pi);
            buf.I1(end+1) = I1;
            buf.I2(end+1) = I2;
            buf.Ipack(end+1) = Ipack;
            buf.Vpack(end+1) = state.Vpack;
            buf.SOC(end+1) = state.SOC;
            
            % update numeric displays
            set(h.lblRPM1,'String',sprintf('%.0f',buf.rpm1(end)));
            set(h.lblRPM2,'String',sprintf('%.0f',buf.rpm2(end)));
            set(h.lblI1,'String',sprintf('%.1f',I1));
            set(h.lblI2,'String',sprintf('%.1f',I2));
            set(h.lblV,'String',sprintf('%.2f',state.Vpack));
            set(h.lblSoC,'String',sprintf('%.1f',state.SOC*100));
            updateDisplay();
            
            % update plots every few steps for performance
            if mod(sim.idx,4)==0
                try
                    set(h.p1(1),'XData',buf.t,'YData',buf.rpm1);
                    set(h.p1(2),'XData',buf.t,'YData',buf.rpm2);
                    set(h.p2(1),'XData',buf.t,'YData',buf.I1);
                    set(h.p2(2),'XData',buf.t,'YData',buf.I2);
                    set(h.p3,'XData',buf.t,'YData',buf.Ipack);
                    set(h.p4,'XData',buf.t,'YData',buf.SOC);
                    drawnow limitrate;
                catch
                    % axes may be cleared on reset; ignore and continue
                end
            end
            
            % advance time and index and safety break
            sim.t = sim.t + dt;
            sim.idx = sim.idx + 1;
            if sim.idx > sim.maxSteps
                stopSim();
                warning('Simulation reached max steps, stopping.');
            end
            
            % small pause to keep UI responsive (approx real-time pacing)
            pause(0.001);
        end
        toc;
    end

% cleanup when figure closes
set(h.fig,'CloseRequestFcn',@onFigClose);

    function onFigClose(src,~)
        sim.running = false;
        delete(src);
    end

end
