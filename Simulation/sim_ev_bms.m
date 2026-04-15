% sim_ev_bms.m
% Time-domain simulation of a 36V VRLA pack + simple BMS (precharge, contactor,
% undervoltage, overcurrent, overtemp) powering two 36V motor controllers.
%
% Run: save as sim_ev_bms.m and run in MATLAB.
% Author: ChatGPT (adapted for user's EV BMS design)
clear; close all; clc;

%% ---------------- Simulation parameters ----------------
dt = 0.01;                % time step (s)
Tsim = 60;                % total sim time (s)
t = 0:dt:Tsim;

% Battery pack (3 x 12V VRLA, 7.2 Ah)
Ncells = 3;
C_Ah = 7.2;               % Ah per battery
C_As = C_Ah * 3600;       % coulomb-equivalent (A·s)
Rint_cell = 0.03;         % internal resistance per cell (Ohm) - tuneable
Rint_pack = Rint_cell * Ncells;

% Open-circuit voltage model per cell (simple linear vs SOC)
OCV_full = 12.6;          % V per cell at 100% SOC
OCV_empty = 11.8;         % V per cell at 0% SOC

% Thermal model (per cell)
mass_eq = 1.0;            % equivalent mass (kg) for heat capacity estimation (tune)
cp = 900;                 % J/(kg*K) (~lead-acid effective) -> Cth = mass*cp
Cth = mass_eq * cp;       % J/K per cell (thermal capacitance)
h_loss = 1.5;             % W/K convective cooling coefficient to ambient (tune)
T_ambient = 25;           % ambient ºC
T_init = 25;              % initial battery temp (ºC)

% Motor/controllers
P_nominal_each = 350;     % W each motor rated
V_nominal = 36;           % nominal pack voltage
I_nominal_each = P_nominal_each / V_nominal; % ~9.7 A
I_peak_each = 80;         % peak inrush per motor (for hard accel) - tune

% Throttle profile (0..1) - user can change this
throttle = zeros(size(t));
% Example: ramp up to 0.8 at 2s, hold, then spike at 10s, then moderate
for k=1:length(t)
    if t(k) < 2
        throttle(k) = 0 + 0.4*(t(k)/2); % gentle ramp to 0.4
    elseif t(k) < 8
        throttle(k) = 0.6;             % cruise
    elseif t(k) < 8.2
        throttle(k) = 1.0;             % short accel spike
    elseif t(k) < 20
        throttle(k) = 0.8;
    elseif t(k) < 30
        throttle(k) = 0.2;             % light regen or idle
    else
        throttle(k) = 0.0;             % off
    end
end

% Pre-charge
use_precharge = true;
R_pre = 68;                % pre-charge resistor (Ohm)
pre_min_time = 1.0;        % minimum pre-charge (s)
pre_timeout = 6.0;         % max pre-charge wait (s)

% BMS thresholds
V_cutoff = 35.0;           % pack undervoltage cutoff (V) -> trip
V_reconnect = 36.8;        % reconnect threshold (V) -> must hold for t_reconnect_hold
t_reconnect_hold = 10.0;   % seconds of stable voltage required to reconnect
I_trip = 80;               % sustained overcurrent threshold (A)
I_instant_trip = 120;      % instant big spike trip threshold (A)
I_trip_duration = 0.25;    % sustained overcurrent duration to trip (s)
T_trip = 55.0;             % over-temperature trip (ºC)
T_warn = 50.0;             % warning temp (ºC)

% Contactors/fuse representation
contactor_closed = false;
precharge_active = false;

%% ---------------- State & storage init ----------------
nSteps = length(t);
Vpack = zeros(1,nSteps);
I_pack = zeros(1,nSteps);    % total pack current (positive = discharge)
I_motor1 = zeros(1,nSteps);
I_motor2 = zeros(1,nSteps);
SOC = ones(1,nSteps);        % normalized SOC (per cell same assumed)
SOC(1) = 1.0;
Vcell_ocv = zeros(1,nSteps);
cell_temp = ones(Ncells,nSteps) * T_init;
contactor_state = false(1,nSteps);
precharge_state = false(1,nSteps);
fault_code = zeros(1,nSteps); % 0=ok, 1=undervolt,2=overcurrent,3=overtemp,4=instant overcurrent
fault_time = NaN;

% Auxiliary counters & timers
overcurrent_timer = 0;
reconnect_timer = 0;
precharge_timer = 0;
precharge_start_time = NaN;

% initial pack voltage (OCV)
Vcell_ocv(1) = OCV_full;
Vpack(1) = Ncells * Vcell_ocv(1);

%% ---------------- Simulation loop ----------------
for k = 2:nSteps
    % ----- battery open-circuit voltage from SOC (linear model) -----
    % per-cell OCV interpolated between empty and full
    Vcell_ocv(k) = OCV_empty + (OCV_full - OCV_empty) * SOC(k-1);
    V_oc_pack = Ncells * Vcell_ocv(k);
    
    % ----- determine motor currents from throttle and instantaneous effects -----
    thr = throttle(k);
    % Base continuous current per motor scale by throttle and pack voltage:
    % I_cont = throttle * I_nominal (scaled by V_nominal/Vpack to model drop)
    I_cont_each = thr * I_nominal_each * (V_nominal / max(1e-3, V_oc_pack));
    % Add a surge/inrush component for brief accel pulses: proportional to throttle^2
    I_surge_each = I_peak_each * (thr^2) * ( (t(k) > 7.98 && t(k) < 8.22) * 1 + 0 ); % example short spike at t~8
    % Also small dynamics: if throttle rising fast, add transient
    if throttle(k) - throttle(max(1,k-1)) > 0.05
        I_surge_each = I_surge_each + 10*(throttle(k)-throttle(max(1,k-1)))/dt;
    end
    I_motor1(k) = min(I_cont_each + I_surge_each, 500); % clamp
    I_motor2(k) = min(I_cont_each + I_surge_each, 500);
    
    % total pack current (sum)
    I_total_no_contactor = I_motor1(k) + I_motor2(k); % current demanded by motors
    % If contactor open, motors draw zero
    if ~contactor_closed
        I_total_no_contactor = 0;
    end
    
    % ----- pre-charge behavior: when precharge active, current flows through precharge resistor
    if use_precharge && precharge_active && ~contactor_closed
        % current through precharge resistor to load (approx Vpack / (R_pre + Rload_internal))
        % Model load internal as motor controllers input impedance is low; approximate current limited by resistor
        I_pre = V_oc_pack / R_pre; % simple
        I_total = I_pre;
    else
        I_total = I_total_no_contactor;
    end
    
    % ----- battery terminal voltage under load (simple Thevenin) -----
    V_term = V_oc_pack - I_total * Rint_pack;
    
    % If contactor open, pack terminal saw no load (but precharge may apply)
    if ~contactor_closed && ~precharge_active
        V_term = V_oc_pack;
    end
    
    % ----- Update SOC (discharge) -----
    % integrate current if discharging; if I_total negative (regen) treat accordingly
    dQ = I_total * dt; % A·s (C)
    SOC(k) = max(0, SOC(k-1) - dQ / C_As);
    
    % ----- Thermal model: battery heating due to I^2*Rint distributed across cells -----
    P_loss_pack = (I_total^2) * Rint_pack; % W dissipated by internal resistance
    P_loss_per_cell = P_loss_pack / Ncells;
    for c = 1:Ncells
        % dT = (P_in - h*(T-Ta))/Cth * dt
        dT = (P_loss_per_cell - h_loss*(cell_temp(c,k-1)-T_ambient)) / Cth * dt;
        cell_temp(c,k) = cell_temp(c,k-1) + dT;
    end
    
    % ----- BMS protection checks & state transitions -----
    % a) undervoltage trip (pack terminal measured by BMS)
    trip = false;
    if V_term <= V_cutoff
        fault_code(k) = 1; % undervolt
        trip = true;
        if isnan(fault_time), fault_time = t(k); end
    end
    
    % b) over-temperature trip (any cell)
    if any(cell_temp(:,k) >= T_trip)
        fault_code(k) = 3; % overtemp
        trip = true;
        if isnan(fault_time), fault_time = t(k); end
    end
    
    % c) overcurrent checks (only valid when contactor is closed and not in precharge)
    if contactor_closed && ~precharge_active
        if I_total >= I_instant_trip
            fault_code(k) = 4; % instant overcurrent
            trip = true;
            if isnan(fault_time), fault_time = t(k); end
        elseif I_total >= I_trip
            overcurrent_timer = overcurrent_timer + dt;
            if overcurrent_timer >= I_trip_duration
                fault_code(k) = 2; % sustained overcurrent
                trip = true;
                if isnan(fault_time), fault_time = t(k); end
            end
        else
            overcurrent_timer = 0;
        end
    else
        overcurrent_timer = 0;
    end
    
    % If trip occurred, open contactor (force open) and clear precharge
    if trip
        contactor_closed = false;
        precharge_active = false;
    end
    
    % ----- Reconnect logic for undervoltage: auto-reconnect when V_term >= V_reconnect for t_reconnect_hold -----
    if ~contactor_closed && fault_code(k) == 1 % only for undervolt
        if V_term >= V_reconnect
            reconnect_timer = reconnect_timer + dt;
            if reconnect_timer >= t_reconnect_hold
                % attempt reconnect via precharge sequence
                precharge_active = true;
                precharge_start_time = t(k);
                % after pre_min_time or until pre_timeout then close contactor
            end
        else
            reconnect_timer = 0;
        end
    end
    
    % Precharge timing and automatic contactor close:
    if precharge_active && ~contactor_closed
        precharge_timer = t(k) - precharge_start_time;
        % estimate load side voltage as V_oc_pack * (Rint_pack / (R_pre + Rint_pack)) - rough proxy
        V_load_side = V_oc_pack * (Rint_pack / (R_pre + Rint_pack));
        % we use a simpler rule: after pre_min_time, close contactor
        if precharge_timer >= pre_min_time
            contactor_closed = true;      % main closes (bypass resistor)
            precharge_active = false;
            precharge_timer = 0;
            precharge_start_time = NaN;
        elseif precharge_timer >= pre_timeout
            % timeout: still close (or fail) - here we close
            contactor_closed = true;
            precharge_active = false;
            precharge_timer = 0;
            precharge_start_time = NaN;
        end
    end
    
    % Manual start logic: in this sim, when time=0 we start precharge and close contactor after pre_min_time
    if k == 2
        % simulate user pressing start at t=0.1s
        precharge_active = true;
        precharge_start_time = t(k);
    end
    
    % If there was a fault other than undervolt and we want manual reset, do nothing (contactor remains open).
    % For simplicity we implement auto-reconnect only for undervolt.
    
    % Record values
    Vpack(k) = V_term;
    I_pack(k) = I_total;
    contactor_state(k) = contactor_closed;
    precharge_state(k) = precharge_active;
    
    % propagate fault_code forward if previously latched
    if fault_code(k) == 0 && k>1
        fault_code(k) = fault_code(k-1);
    end
end

%% ---------------- Plot results ----------------
figure('Name','EV BMS Simulation Results','Position',[60 60 1200 800]);

subplot(4,1,1);
plot(t, Vpack, 'b','LineWidth',1.5); hold on;
yline(V_cutoff,'r--','Cutoff','LabelVerticalAlignment','bottom');
yline(V_reconnect,'g--','Reconnect');
xlabel('Time (s)'); ylabel('Pack Voltage (V)'); grid on;
title('Pack Terminal Voltage'); ylim([min(Vpack)-1, max(Vpack)+1]);

subplot(4,1,2);
plot(t, I_pack, 'k','LineWidth',1.5); hold on;
yline(I_trip,'m--','I_{sustained}'); yline(I_instant_trip,'r--','I_{instant}');
xlabel('Time (s)'); ylabel('Pack Current (A)'); grid on;
title('Pack Current (discharge positive)');

subplot(4,1,3);
plot(t, cell_temp(1,:),'r','LineWidth',1.2); hold on;
if Ncells>1
    plot(t, cell_temp(2,:),'g','LineWidth',1.2);
    plot(t, cell_temp(3,:),'b','LineWidth',1.2);
end
yline(T_trip,'r--','T_{trip}');
yline(T_warn,'y--','T_{warn}');
xlabel('Time (s)'); ylabel('Cell Temp (°C)'); grid on;
legendStrings = arrayfun(@(c) sprintf('Cell %d',c), 1:Ncells, 'UniformOutput', false);
legend(legendStrings,'Location','best');

subplot(4,1,4);
plot(t, double(contactor_state),'LineWidth',2); hold on;
plot(t, double(precharge_state),'LineWidth',1.5);
stairs(t, fault_code,'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Contact/State'); grid on;
legend('Contactor closed (1=closed)','Precharge active','Fault code (0=ok,1=UV,2=OC,3=OT,4=inst OC)','Location','eastoutside');

% Annotate first fault time if any
if ~isnan(fault_time)
    subplot(4,1,1);
    xline(fault_time,'r--','LineWidth',1.2);
    subplot(4,1,2);
    xline(fault_time,'r--','LineWidth',1.2);
    subplot(4,1,3);
    xline(fault_time,'r--','LineWidth',1.2);
end

sgtitle('EV BMS Simulation: Pack voltage, current, temps, contactor & fault states');

%% ---------------- Summary printout ----------------
fprintf('Simulation finished. Final SOC: %.3f (%%)\n', SOC(end)*100);
if ~isnan(fault_time)
    fprintf('First fault occurred at t = %.2f s, fault code = %d\n', fault_time, fault_code(find(~isnan(fault_time),1)));
else
    fprintf('No faults occurred during simulation.\n');
end

%% ---------------- How to tweak ----------------
% - Change throttle profile near the top to simulate different driving behaviors.
% - Set Rint_cell higher for older/weaker batteries.
% - Adjust I_peak_each to produce larger inrush currents.
% - Adjust pre-charge R_pre to model faster/slower pre-charge behavior.
% - Enable/disable auto-reconnect logic or change t_reconnect_hold.

