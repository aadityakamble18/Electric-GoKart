function ev_sim_long_soc_fixed()
% ev_sim_long_soc_fixed
% Long EV simulation with Ke/Kt computed from desired no-load RPM.
% Save as ev_sim_long_soc_fixed.m and run.

close all; clc;

%% ---------- Simulation control ----------
Tsim = 360000;       % total simulation time [s] (1 hour)
dt   = 0.1;        % timestep [s]
t = 0:dt:Tsim;
N = numel(t);

%% ---------- Battery pack ----------
V_nom = 36.0;      % nominal pack voltage [V]
Rb    = 0.02;      % pack internal resistance [ohm]
Ah    = 7.2;       % pack capacity (your battery)
Q     = Ah * 3600; % capacity in Coulombs
SoC = zeros(1,N); SoC(1) = 1.0;  % start fully charged

%% ---------- Fuses & contactor (kept permissive) ----------
main_fuse_rating = 200; main_fuse_time_limit = 0.5;
controller_fuse_rating = 150; controller_fuse_time_limit = 0.5;
main_fuse_blown = false; fuse1_blown = false; fuse2_blown = false;
main_fuse_timer = 0; fuse1_timer = 0; fuse2_timer = 0;
key_on = true; contactor_closed = key_on;

%% ---------- Motor/controller params ----------
% Controller current limits
I_cont_max = 120;    % per-controller steady current limit [A]
I_cont_peak = 250;   % used to set R_eq

% Choose a realistic desired no-load RPM at nominal voltage:
noload_rpm = 4500;   % change if you expect a different motor speed

% Compute Ke and Kt consistently (SI units)
omega0 = noload_rpm * 2*pi/60;        % rad/s
Ke = V_nom / omega0;                  % V per (rad/s)
Kt = Ke;                              % N*m/A (SI-consistent)

% Mechanical parameters
J = 0.02;            % inertia [kg*m^2]
b = 0.0008;          % viscous damping
load_torque = 0.6;   % external torque (road + friction)

% Display chosen constants
fprintf('Using computed motor constants from noload_rpm = %d rpm\n', noload_rpm);
fprintf('Computed Ke = %.5f V/(rad/s), Kt = %.5f N*m/A\n', Ke, Kt);

%% ---------- Throttle profile (long test) ----------
throttle = zeros(1,N);
warmRampEnd = round(60/dt);   % 60s ramp
throttle(1:warmRampEnd) = linspace(0,0.8,warmRampEnd);
sustainStart = warmRampEnd + 1;
sustainEnd = min(N, sustainStart + round(1800/dt)); % ~30 minutes sustained
throttle(sustainStart:sustainEnd) = 0.8;  % sustained draw
cruiseStart = sustainEnd + 1;
cruiseEnd = min(N, cruiseStart + round(1200/dt)); % ~20 min cruise
throttle(cruiseStart:cruiseEnd) = 0.45;
for k = cruiseEnd+1 : round(0.05*N) : N
    idx = min(N, k:(k+round(5/dt)));
    throttle(idx) = 0.9; % short pulses
end
throttle = max(0, min(1, throttle));

%% ---------- Preallocate state arrays ----------
omega1 = zeros(1,N); omega2 = zeros(1,N);   % rad/s
Ia1 = zeros(1,N); Ia2 = zeros(1,N);
Ipack = zeros(1,N); Vpack = zeros(1,N);
Vpack(1) = V_nom;

rpm_from_omega = @(w) w*60/(2*pi);

%% ---------- Main simulation loop (Euler) ----------
for k = 1:N-1
    thr = throttle(k);

    % Bus voltage (if contactor closed and main fuse OK)
    if ~contactor_closed || main_fuse_blown
        Va = 0;
    else
        Va = Vpack(k);
    end

    % Equivalent winding resistance chosen to allow peak currents at stall
    R_eq = max(0.003, V_nom / I_cont_peak);

    % Back-EMF (V)
    Vbemf1 = Ke * omega1(k);
    Vbemf2 = Ke * omega2(k);

    % Controller effective drive voltage (simple PWM-as-voltage model)
    Vdrive1 = Va * thr;
    Vdrive2 = Va * thr;

    % Currents (clamped by controller limit)
    I1 = max(0, min(I_cont_max, (Vdrive1 - Vbemf1)/R_eq));
    I2 = max(0, min(I_cont_max, (Vdrive2 - Vbemf2)/R_eq));

    % Respect blown fuses
    if fuse1_blown, I1 = 0; end
    if fuse2_blown, I2 = 0; end

    % Totals and battery sag
    I_total = I1 + I2;
    Ipack(k) = I_total;
    Vpack(k) = V_nom - I_total * Rb;

    % SoC (Coulomb counting)
    dQ = I_total * dt; % A * s = C
    SoC(k+1) = max(0, SoC(k) - dQ / Q);

    % Mechanical dynamics: torque = Kt * I
    Tm1 = Kt * I1;
    Tm2 = Kt * I2;
    domega1 = (Tm1 - load_torque - b * omega1(k)) / J;
    domega2 = (Tm2 - load_torque - b * omega2(k)) / J;
    omega1(k+1) = max(0, omega1(k) + domega1 * dt);
    omega2(k+1) = max(0, omega2(k) + domega2 * dt);

    Ia1(k) = I1; Ia2(k) = I2;

    % Fuse logic (permissive for long sim)
    if I_total > main_fuse_rating
        main_fuse_timer = main_fuse_timer + dt;
    else
        main_fuse_timer = max(0, main_fuse_timer - dt*2);
    end
    if main_fuse_timer >= main_fuse_time_limit
        main_fuse_blown = true; contactor_closed = false;
        fprintf('Main fuse blown at t=%.1fs\n', t(k));
    end

    if I1 > controller_fuse_rating
        fuse1_timer = fuse1_timer + dt;
    else
        fuse1_timer = max(0, fuse1_timer - dt*2);
    end
    if fuse1_timer >= controller_fuse_time_limit && ~fuse1_blown
        fuse1_blown = true; fprintf('Fuse1 blown at t=%.1fs\n', t(k));
    end

    if I2 > controller_fuse_rating
        fuse2_timer = fuse2_timer + dt;
    else
        fuse2_timer = max(0, fuse2_timer - dt*2);
    end
    if fuse2_timer >= controller_fuse_time_limit && ~fuse2_blown
        fuse2_blown = true; fprintf('Fuse2 blown at t=%.1fs\n', t(k));
    end

    % Low-voltage cutoff: open contactor to protect battery
    if Vpack(k) < 30 && contactor_closed
        contactor_closed = false;
        fprintf('Contactor opened at t=%.1fs due to low Vpack=%.2f V\n', t(k), Vpack(k));
    end

    % Periodic progress print (every 300s)
    if mod(k, round(300/dt))==0
        fprintf('t=%.0f s | Vpack=%.2f V | Ipack=%.2f A | SoC=%.1f%%\n', t(k), Vpack(k), Ipack(k), SoC(k)*100);
    end
end

% finalize last values
Ipack(end) = Ia1(end) + Ia2(end);
Vpack(end) = V_nom - Ipack(end)*Rb;
rpm1 = rpm_from_omega(omega1);
rpm2 = rpm_from_omega(omega2);

%% ---------- Plots ----------
figure('Name','Long EV Simulation Results (fixed Ke/Kt)','NumberTitle','off','Units','normalized','Position',[0.05 0.05 0.9 0.8]);

subplot(3,2,1); plot(t, throttle,'r','LineWidth',1.2); ylim([-0.05 1.05]); grid on; title('Throttle'); xlabel('Time (s)');
subplot(3,2,2); plot(t, Vpack,'b','LineWidth',1.2); grid on; title('Pack Voltage [V]'); xlabel('Time (s)');
ylim([min(Vpack)-1, V_nom+2]);

subplot(3,2,3); plot(t, Ia1,'LineWidth',1.0); hold on; plot(t, Ia2,'LineWidth',1.0); hold off; grid on; legend('I1','I2'); title('Controller Currents [A]'); xlabel('Time (s)');
subplot(3,2,4); plot(t, Ipack,'k','LineWidth',1.2); grid on; title('Pack Current [A]'); xlabel('Time (s)');

subplot(3,2,5); plot(t, rpm1,'LineWidth',1.0); hold on; plot(t, rpm2,'LineWidth',1.0); hold off; grid on; legend('RPM1','RPM2'); title('Motor RPM'); xlabel('Time (s)');
subplot(3,2,6); plot(t, SoC,'g','LineWidth',1.2); grid on; title('State of Charge (SoC)'); xlabel('Time (s)'); ylim([-0.05 1.05]);

sgtitle('Long EV Pack Simulation (1 hour) — Ke/Kt fixed from desired no-load RPM','FontWeight','bold');

%% ---------- Final summary ----------
fprintf('\nSimulation complete.\nFinal pack voltage: %.2f V\nFinal SoC: %.2f%%\nMax pack current: %.2f A\nFuse states: main=%d, fuse1=%d, fuse2=%d\n', ...
    Vpack(end), SoC(end)*100, max(Ipack), main_fuse_blown, fuse1_blown, fuse2_blown);

end

