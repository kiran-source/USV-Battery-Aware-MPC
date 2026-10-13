function [T_cmd, N_cmd, state_pid] = PID_battery_aware(u_ref, psi_ref, u_meas, psi_meas, r_meas, SOC_cur, dt, state_pid, p)
    % PID_BATTERY_AWARE Battery-Aware PID Speed & Heading Controller
    %
    % Same dual-loop PID structure as PID_controller, but with:
    %   1. Adaptive gain scheduling based on Battery SOC
    %   2. Dynamic thrust capping when SOC is low
    %   3. Speed reference derating at low SOC levels
    %
    % Inputs:
    %   u_ref     - Target surge velocity [m/s]
    %   psi_ref   - Target heading angle [rad]
    %   u_meas    - Measured surge velocity [m/s]
    %   psi_meas  - Measured heading angle [rad]
    %   r_meas    - Measured yaw rate [rad/s]
    %   SOC_cur   - Current battery State of Charge [%]
    %   dt        - Timestep [s]
    %   state_pid - Struct containing integrators and previous errors
    %   p         - Full parameter struct
    %
    % Outputs:
    %   T_cmd     - Total surge thrust command [N]
    %   N_cmd     - Differential yaw moment command [N*m]
    %   state_pid - Updated integrator states
    
    if isempty(state_pid)
        state_pid.int_eu    = 0.0;
        state_pid.prev_eu   = 0.0;
        state_pid.int_epsi  = 0.0;
        state_pid.prev_epsi = 0.0;
    end
    
    %% 1. BATTERY-AWARE SPEED REFERENCE DERATING
    % Reduce target speed progressively as SOC drops below economy threshold
    SOC_economy = p.battery.soc_economy_pct;   % 50%
    SOC_reserve = p.battery.soc_reserve_pct;    % 20%
    
    if SOC_cur <= SOC_reserve
        % Below reserve: emergency crawl
        soc_scale = 0.3;
    elseif SOC_cur <= SOC_economy
        % Between reserve and economy: linear derating
        soc_scale = 0.3 + 0.7 * ((SOC_cur - SOC_reserve) / (SOC_economy - SOC_reserve));
    else
        % Above economy threshold: full speed
        soc_scale = 1.0;
    end
    
    u_ref_derated = u_ref * soc_scale;
    
    %% 2. ADAPTIVE PID GAINS (Scheduled by SOC)
    % Reduce proportional gain at low SOC to prevent aggressive thrust
    Kp_u = 140.0 * soc_scale;
    Ki_u = 18.0 * soc_scale;
    Kd_u = 12.0;
    
    % Heading gains remain unchanged for safety
    Kp_psi = 65.0;
    Ki_psi = 4.0;
    Kd_psi = 35.0;
    
    %% 3. SPEED ERROR & PID CONTROL (with derated reference)
    e_u = u_ref_derated - u_meas;
    state_pid.int_eu = state_pid.int_eu + e_u * dt;
    state_pid.int_eu = min(max(state_pid.int_eu, -5.0), 5.0);
    
    der_eu = (e_u - state_pid.prev_eu) / dt;
    state_pid.prev_eu = e_u;
    
    T_raw = Kp_u * e_u + Ki_u * state_pid.int_eu + Kd_u * der_eu;
    
    %% 4. BATTERY-AWARE THRUST CAPPING
    % Dynamically limit maximum thrust based on SOC
    if SOC_cur <= SOC_reserve
        T_max_allowed = 80.0;   % Severely limited
    elseif SOC_cur <= SOC_economy
        T_max_allowed = 80.0 + (p.usv.prop.T_max_tot - 80.0) * ...
            ((SOC_cur - SOC_reserve) / (SOC_economy - SOC_reserve));
    else
        T_max_allowed = p.usv.prop.T_max_tot;
    end
    
    T_cmd = min(max(T_raw, p.usv.prop.T_min_tot), T_max_allowed);
    
    %% 5. HEADING ERROR & PID CONTROL (unchanged for safety)
    e_psi = wrapToPi(psi_ref - psi_meas);
    state_pid.int_epsi = state_pid.int_epsi + e_psi * dt;
    state_pid.int_epsi = min(max(state_pid.int_epsi, -1.0), 1.0);
    
    N_raw = Kp_psi * e_psi + Ki_psi * state_pid.int_epsi - Kd_psi * r_meas;
    N_cmd = min(max(N_raw, -p.usv.prop.N_max), p.usv.prop.N_max);
end
