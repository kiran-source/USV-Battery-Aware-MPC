%% ========================================================================
%  Battery-Aware Model Predictive Speed Control for an Autonomous
%  Surface Vehicle (USV)  --  vs. conventional PID speed control
%
%  Group Project: Battery-Aware MPC Speed Control for USVs
%
%  This script:
%    1. Defines a mathematical (surge) model of the USV.
%    2. Estimates power consumption and battery State of Charge (SOC).
%    3. Implements a receding-horizon (MPC) speed controller that is
%       AWARE of battery SOC, remaining distance and the energy needed
%       to safely return to base.
%    4. Implements a conventional PID speed controller (no battery
%       awareness) as the baseline for comparison.
%    5. Simulates a full mission: Base -> Destination -> Base.
%    6. Plots battery percentage, speed tracking, distance, power and
%       a final comparison summary.
%
%  Run with: MATLAB R2016b or later (uses local functions in a script).
%  No toolboxes are required (custom grid-search MPC, no Optimization
%  Toolbox needed).
% =========================================================================

clear; clc; close all;

%% ---------------------- 1. VEHICLE / PHYSICAL MODEL --------------------
% Surge (forward motion) dynamics of the USV, from Newton's 2nd law:
%
%       m * dv/dt = T(t) - ( c1*v(t) + c2*v(t)^2 )                 ...(1)
%
%   m   : effective mass (hull mass + hydrodynamic added mass)  [kg]
%   T   : propeller thrust force (control input)                [N]
%   c1  : linear (viscous) drag coefficient               [N/(m/s)]
%   c2  : quadratic (form) drag coefficient              [N/(m/s)^2]
%
% Kinematics (position along the mission track, x = 0 at Base):
%
%       dx/dt = v(t)                                                ...(2)
%
% Propulsion electrical power drawn from the battery:
%
%       P_thrust(t) = T(t)*v(t)/eta_p  +  k_loss * T(t)^2           ...(3)
%       P(t)        = P_thrust(t) + P_hotel                         ...(4)
%
%   eta_p   : propeller/driveline efficiency                 [-]
%   k_loss  : electrical/mechanical loss coefficient   [W/N^2]
%             (captures bollard-pull / stationary thrust losses)
%   P_hotel : "hotel" load - sensors, comms, computer          [W]
%
% Battery State of Charge (SOC) dynamics (Coulomb/energy counting):
%
%       SOC(t+dt) = SOC(t) - [ P(t)*dt / (3600*E_cap) ] * 100       ...(5)
%
%   E_cap : usable battery energy capacity                    [Wh]
%
% Specific energy of travel (energy needed per metre at speed v),
% obtained from the steady-state (dv/dt = 0) thrust in eq.(1):
%
%       T_ss(v)   = c1*v + c2*v^2                                   ...(6)
%       P_ss(v)   = T_ss(v)*v/eta_p + k_loss*T_ss(v)^2 + P_hotel    ...(7)
%       e(v)      = P_ss(v) / v          [W / (m/s)] = [J/m]        ...(8)
%
% This e(v) is what the battery-aware supervisor uses to check whether
% the remaining energy is enough to (a) reach the destination and
% (b) safely return to base.
% =========================================================================

p.m        = 300;      % effective mass [kg]
p.c1       = 15;       % linear drag [N/(m/s)]
p.c2       = 25;       % quadratic drag [N/(m/s)^2]
p.eta_p    = 0.70;     % propulsion efficiency
p.k_loss   = 0.02;     % electrical loss coeff [W/N^2]
p.P_hotel  = 40;       % hotel load [W]
p.E_cap_Wh = 650;      % battery usable capacity [Wh]  (~ a small 24V/27Ah pack)
p.T_max    = 400;      % max thrust [N]
p.v_min    = 0.0;      % min commandable speed [m/s]
p.v_max    = 2.2;      % max commandable speed [m/s]

%% ---------------------- 2. MISSION DEFINITION ---------------------------
D_total   = 5000;      % one-way distance, Base -> Destination [m]
dt        = 5;         % simulation / control time step [s]
T_end_max = 6000;       % hard safety cap on simulated steps
v_ref_PID = 2.0;        % fixed reference speed commanded by the PID [m/s]

% Battery-aware supervisor thresholds
SOC_reserve_margin = 1.30;  % required energy safety margin (30%) to guarantee safe completion
SOC_economy_trig   = 1.70;  % below this margin -> switch to ECONOMY speed
SOC_emergency      = 8;     % [%] hard emergency threshold

v_eco = 1.0;   % efficient / economical cruise speed [m/s] used for
               % energy-feasibility checks and ECONOMY / RETURN phases
v_emg = 0.4;   % crawl speed used in EMERGENCY mode [m/s]

%% ---------------------- 3. RUN BATTERY-AWARE MPC CONTROLLER -------------
fprintf('Running Battery-Aware MPC simulation...\n');
mpc = run_mission(p, D_total, dt, T_end_max, 'MPC', v_ref_PID, ...
                   SOC_reserve_margin, SOC_economy_trig, SOC_emergency, ...
                   v_eco, v_emg);

%% ---------------------- 4. RUN CONVENTIONAL PID CONTROLLER --------------
fprintf('Running conventional PID simulation...\n');
pid = run_mission(p, D_total, dt, T_end_max, 'PID', v_ref_PID, ...
                   SOC_reserve_margin, SOC_economy_trig, SOC_emergency, ...
                   v_eco, v_emg);

%% ---------------------- 5. RESULTS SUMMARY -------------------------------
fprintf('\n================= MISSION SUMMARY =================\n');
print_summary('Battery-Aware MPC', mpc);
print_summary('Conventional PID ', pid);
fprintf('=====================================================\n');

%% ---------------------- 6. EXPORT CSV (for the Python dashboard) --------
writetable(struct2table(mpc), 'usv_mpc_results.csv');
writetable(struct2table(pid), 'usv_pid_results.csv');
fprintf('\nSaved usv_mpc_results.csv and usv_pid_results.csv\n');

%% ---------------------- 7. PLOTS -----------------------------------------

% ---- Figure 1: Battery SOC (%) vs time, MPC vs PID ----------------------
figure('Name','Battery SOC (%) vs Time','Color','w');
plot(mpc.t/60, mpc.SOC, 'g-', 'LineWidth', 2); hold on;
plot(pid.t/60, pid.SOC, 'r--', 'LineWidth', 2);
yline(SOC_emergency, ':k', 'Emergency threshold', 'LineWidth', 1.2);
xlabel('Time [min]'); ylabel('Battery State of Charge [%]');
title('Battery Percentage Remaining vs Time');
legend('Battery-Aware MPC','Conventional PID','Location','southwest');
grid on; ylim([0 105]);

% ---- Figure 2: Speed & Distance tracking --------------------------------
figure('Name','Speed and Distance Tracking','Color','w');
subplot(2,1,1);
plot(mpc.t/60, mpc.v, 'g-', 'LineWidth', 1.6); hold on;
plot(pid.t/60, pid.v, 'r--', 'LineWidth', 1.6);
xlabel('Time [min]'); ylabel('Speed [m/s]');
title('USV Speed Profile'); legend('MPC (adaptive speed)','PID (fixed reference)');
grid on;

subplot(2,1,2);
plot(mpc.t/60, mpc.x, 'g-', 'LineWidth', 1.6); hold on;
plot(pid.t/60, pid.x, 'r--', 'LineWidth', 1.6);
yline(D_total, 'k:', 'Destination', 'LineWidth', 1.1);
yline(0, 'k:', 'Base', 'LineWidth', 1.1);
xlabel('Time [min]'); ylabel('Position along track [m]');
title('Position: Base \rightarrow Destination \rightarrow Base');
legend('MPC','PID','Location','best'); grid on;

% ---- Figure 3: Power draw and cumulative energy -------------------------
figure('Name','Power and Energy Consumption','Color','w');
subplot(2,1,1);
plot(mpc.t/60, mpc.P, 'g-', 'LineWidth', 1.4); hold on;
plot(pid.t/60, pid.P, 'r--', 'LineWidth', 1.4);
xlabel('Time [min]'); ylabel('Power draw [W]');
title('Instantaneous Battery Power Draw'); legend('MPC','PID'); grid on;

subplot(2,1,2);
E_mpc_cum = cumtrapz(mpc.t, mpc.P)/3600;   % Wh
E_pid_cum = cumtrapz(pid.t, pid.P)/3600;   % Wh
plot(mpc.t/60, E_mpc_cum, 'g-', 'LineWidth', 1.6); hold on;
plot(pid.t/60, E_pid_cum, 'r--', 'LineWidth', 1.6);
xlabel('Time [min]'); ylabel('Cumulative energy used [Wh]');
title('Cumulative Battery Energy Consumption'); legend('MPC','PID','Location','northwest');
grid on;

% ---- Figure 4: Mission mode timeline (MPC only) --------------------------
figure('Name','Battery-Aware Supervisor: Mission Mode','Color','w');
mode_num = mpc.mode_id;   % 1=OUTBOUND(cruise) 2=ECONOMY 3=RETURN 4=EMERGENCY
stairs(mpc.t/60, mode_num, 'b-', 'LineWidth', 1.8);
yticks([1 2 3 4]);
yticklabels({'CRUISE (Outbound)','ECONOMY','RETURN-TO-BASE','EMERGENCY'});
xlabel('Time [min]'); title('Battery-Aware Decision Mode over the Mission');
grid on; ylim([0.5 4.5]);

% ---- Figure 5: Final comparison bar chart --------------------------------
figure('Name','Final Comparison Summary','Color','w');
metrics = {'Mission time [min]','Total energy [Wh]','Final SOC [%]'};
mpc_vals = [mpc.t(end)/60, E_mpc_cum(end), mpc.SOC(end)];
pid_vals = [pid.t(end)/60, E_pid_cum(end), pid.SOC(end)];
b = bar([mpc_vals; pid_vals]');
b(1).FaceColor = [0.2 0.7 0.2];
b(2).FaceColor = [0.8 0.2 0.2];
set(gca,'xticklabel', metrics);
legend('Battery-Aware MPC','Conventional PID','Location','best');
title('Performance Comparison: MPC vs PID'); grid on;

fprintf('\nAll plots generated. Done.\n');


%% =========================================================================
%                            LOCAL FUNCTIONS
% =========================================================================

function out = run_mission(p, D_total, dt, T_end_max, ctrl_type, v_ref_PID, ...
                            SOC_reserve_margin, SOC_economy_trig, SOC_emergency, ...
                            v_eco, v_emg)
% RUN_MISSION simulates one full Base->Destination->Base mission using
% either the battery-aware MPC controller or a conventional PID
% controller, and returns time histories for plotting.

    % --- state init ---
    t = 0; x = 0; v = 0; SOC = 100;
    phase = 1;   % 1=OUTBOUND(cruise) 2=ECONOMY 3=RETURN 4=EMERGENCY
    direction = 1; % +1 heading to destination, -1 heading back to base

    % PID gains / state (only used when ctrl_type == 'PID')
    Kp = 90; Ki = 12; Kd = 6;   % tuned for the dt_sub inner integration loop
    int_e = 0; prev_e = 0;

    % MPC control-move smoothing (low-pass filter on the target speed, so
    % the receding-horizon re-plan each cycle doesn't chatter between two
    % adjacent candidate speeds)
    last_v_ref = 0;
    alpha_smooth = 0.15;

    dt_sub = 0.5;                 % inner physics/PID integration step [s]
    n_sub  = round(dt/dt_sub);    % substeps per outer control step

    % Preallocate log arrays
    N_max = T_end_max;
    T_log   = zeros(N_max,1); t_log = zeros(N_max,1); x_log = zeros(N_max,1);
    v_log   = zeros(N_max,1); SOC_log = zeros(N_max,1); P_log = zeros(N_max,1);
    mode_log = zeros(N_max,1);

    k = 0;
    mission_done = false;

    while ~mission_done && k < N_max
        k = k + 1;

        % ---------------- Battery-aware supervisor (mode selection) -----
        if strcmp(ctrl_type,'MPC')
            dist_to_dest = max(D_total - x, 0);
            if direction > 0
                remaining_round_trip = dist_to_dest + D_total; % to dest + back to base
            else
                remaining_round_trip = x; % distance left to reach base
            end

            % Energy needed at economical speed for the remaining mission
            e_spec  = specific_energy(p, v_eco);          % [J/m]
            E_need_Wh = e_spec * remaining_round_trip / 3600; % [Wh]
            E_have_Wh = SOC/100 * p.E_cap_Wh;

            if SOC <= SOC_emergency
                phase = 4; % EMERGENCY
            elseif E_have_Wh < E_need_Wh * SOC_reserve_margin
                if direction > 0
                    phase = 3; % not enough for round trip -> abort & RETURN
                    direction = -1;
                else
                    phase = 3; % already returning, keep returning
                end
            elseif E_have_Wh < E_need_Wh * SOC_economy_trig
                phase = 2; % ECONOMY (reduce speed to save energy)
            else
                phase = 1; % normal CRUISE
            end

            % Destination reached -> switch to return leg automatically
            if direction > 0 && x >= D_total
                direction = -1;
                if phase == 1, phase = 1; end % continue cruising back
            end
            % Reached base again on the return leg -> mission complete
            if direction < 0 && x <= 0
                x = 0; mission_done = true;
            end

            % ---------------- MPC: constraint-based receding horizon -----
            if ~mission_done
                v_cmd = mpc_select_speed(p, v, x, D_total, direction, phase, ...
                                          SOC, v_eco, v_emg, dt, SOC_reserve_margin);
                % Control-move suppression: low-pass filter the target
                % speed so re-planning each cycle doesn't chatter between
                % two adjacent candidate speeds near a feasibility boundary.
                v_ref = alpha_smooth*v_cmd + (1-alpha_smooth)*last_v_ref;
                last_v_ref = v_ref;
            else
                v_ref = 0;
            end

        else % ---------------- conventional PID (no battery awareness) --
            phase = 1;
            if x >= D_total, direction = -1; end
            if direction < 0 && x <= 0, x = 0; mission_done = true; end
            v_ref = v_ref_PID;
        end

        % ---------------- inner loop: physics + low-level control --------
        % Explicit-Euler integration of the fast PID feedback loop at the
        % coarse outer decision rate (dt) is numerically unstable, so the
        % vehicle dynamics (eqs. 1-5) are integrated at the finer dt_sub.
        P = p.P_hotel;
        for j = 1:n_sub
            if mission_done, break; end

            if strcmp(ctrl_type,'MPC')
                tau_v = 8; % s, first-order response time constant of speed loop
                dv = (v_ref - v)/tau_v * dt_sub;
                v_new = max(v + dv, 0);
                T_cmd = p.m*dv/dt_sub + p.c1*v + p.c2*v^2;
                T_cmd = min(max(T_cmd, 0), p.T_max);
            else
                e = v_ref - v;
                int_e = int_e + e*dt_sub;
                der_e = (e - prev_e)/dt_sub;
                T_cmd = Kp*e + Ki*int_e + Kd*der_e;
                T_cmd = min(max(T_cmd, 0), p.T_max);
                prev_e = e;
                dv = (T_cmd - (p.c1*v + p.c2*v^2))/p.m * dt_sub;
                v_new = max(v + dv, 0);
            end

            % Power & battery update (eqs. 3-5)
            P = T_cmd*v/p.eta_p + p.k_loss*T_cmd^2 + p.P_hotel;
            SOC = max(SOC - (P*dt_sub)/(3600*p.E_cap_Wh)*100, 0);

            % If PID drains the battery completely -> vehicle stranded
            if strcmp(ctrl_type,'PID') && SOC <= 0
                v_new = 0; mission_done = true;
            end

            x = x + direction*v*dt_sub;
            x = min(max(x, 0), D_total*1.001);
            t = t + dt_sub;
            v = v_new;
        end

        % ---------------- log (once per outer control step) --------------
        t_log(k) = t; x_log(k) = x; v_log(k) = v; SOC_log(k) = SOC;
        P_log(k) = P; T_log(k) = T_cmd; mode_log(k) = phase;
    end

    out.t = t_log(1:k); out.x = x_log(1:k); out.v = v_log(1:k);
    out.SOC = SOC_log(1:k); out.P = P_log(1:k); out.T = T_log(1:k);
    out.mode_id = mode_log(1:k);
end


function v_cmd = mpc_select_speed(p, v_cur, x, D_total, direction, phase, SOC, v_eco, v_emg, dt, SOC_reserve_margin)
% MPC_SELECT_SPEED: constraint-based receding-horizon controller.
%
% For a set of candidate cruise speeds (fastest to slowest, restricted to
% the range allowed by the current battery-aware phase), this function:
%   1. Forward-simulates the vehicle N steps ahead (eqs. 1-5) to predict
%      the energy that would be consumed and the position reached.
%   2. Checks whether the PREDICTED remaining battery energy would still
%      be enough (with the safety margin) to complete the rest of the
%      mission at the economical speed v_eco.
%   3. Selects the FASTEST candidate that passes this feasibility check
%      -- i.e. "go as fast as you safely can, no faster."
% This directly implements the problem statement's requirement to
% continuously weigh speed against remaining energy, distance to the
% destination, and the energy needed to safely return to base.

    N = 6;           % prediction horizon steps (30 s look-ahead at dt=5s)
    tau_v = 8;        % speed-loop response time constant [s]

    switch phase
        case 1 % CRUISE - full adaptive range
            v_lo = 0.5; v_hi = p.v_max;
        case 2 % ECONOMY - favor efficient speed, allow a little headroom
            v_lo = 0.4; v_hi = v_eco + 0.4;
        case 3 % RETURN - efficient speed only, mission aborted
            v_lo = 0.3; v_hi = v_eco;
        otherwise % EMERGENCY - crawl
            v_lo = 0; v_hi = v_emg;
    end

    candidates = linspace(v_hi, v_lo, 10);  % fastest first
    E_have = SOC/100 * p.E_cap_Wh;
    e_spec_eco = specific_energy(p, v_eco);

    v_cmd = v_lo; % fallback: best-effort (slowest / most conservative)
    for i = 1:length(candidates)
        vc = candidates(i);
        vp = v_cur; xp = x; E_used = 0;
        for j = 1:N
            dv = (vc - vp)/tau_v*dt;
            vp = max(vp + dv, 0);
            Tp = p.m*dv/dt + p.c1*vp + p.c2*vp^2;
            Tp = min(max(Tp,0), p.T_max);
            Pp = Tp*vp/p.eta_p + p.k_loss*Tp^2 + p.P_hotel;
            E_used = E_used + Pp*dt/3600;   % Wh
            xp = xp + direction*vp*dt;
        end

        E_after = E_have - E_used;
        dist_to_dest_after = max(D_total - xp, 0);
        if direction > 0
            remaining_after = dist_to_dest_after + D_total;
        else
            remaining_after = max(xp, 0);
        end
        E_need_after = e_spec_eco * remaining_after / 3600;   % Wh

        if E_after >= E_need_after * SOC_reserve_margin   % same margin as the supervisor
            v_cmd = vc;   % fastest feasible candidate found -> use it
            break;
        end
    end
end


function e_spec = specific_energy(p, v)
% Specific energy of travel e(v) = P_ss(v)/v  [J per metre], eqs. 6-8.
    v = max(v, 0.05);
    T_ss = p.c1*v + p.c2*v^2;
    P_ss = T_ss*v/p.eta_p + p.k_loss*T_ss^2 + p.P_hotel;
    e_spec = P_ss / v;
end


function print_summary(name, s)
    fprintf('%s: time=%.1f min | energy=%.1f Wh | final SOC=%.1f%% | %s\n', ...
        name, s.t(end)/60, trapz(s.t, s.P)/3600, s.SOC(end), ...
        completion_str(s));
end

function str = completion_str(s)
    if s.SOC(end) <= 0.01
        str = 'STRANDED (battery depleted)';
    elseif s.x(end) <= 1
        str = 'Mission complete (returned to base)';
    else
        str = 'Mission incomplete';
    end
end
