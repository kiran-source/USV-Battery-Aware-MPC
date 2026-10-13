function [tau_ctrl, P_prop, T_L_act, T_R_act] = propulsion_model(T_cmd, N_cmd, u, T_L_prev, T_R_prev, dt, p_usv)
    % PROPULSION_MODEL Differential Electric Thruster Propulsion System
    %
    % Inputs:
    %   T_cmd     - Commanded total surge thrust [N]
    %   N_cmd     - Commanded differential yaw moment [N*m]
    %   u         - Current surge speed [m/s]
    %   T_L_prev  - Previous Left thruster force [N]
    %   T_R_prev  - Previous Right thruster force [N]
    %   dt        - Timestep [s]
    %   p_usv     - USV parameter struct
    %
    % Outputs:
    %   tau_ctrl  - Actual control vector [T_act; 0; N_act] in body frame
    %   P_prop    - Total propulsion electrical power draw [Watts]
    %   T_L_act   - Actual left thruster output [N]
    %   T_R_act   - Actual right thruster output [N]
    
    B = p_usv.prop.B_prop; % Baseline separation [m]
    T_max_ind = p_usv.prop.T_max_ind;
    T_min_ind = p_usv.prop.T_min_ind;
    rate_max  = p_usv.prop.rate_max;
    eta_motor = p_usv.prop.eta_motor;
    k_loss    = p_usv.prop.k_loss;
    
    %% 1. THRUSTER ALLOCATION (Mapping [T, N] -> [T_L, T_R])
    T_L_des = 0.5 * T_cmd - (N_cmd / B);
    T_R_des = 0.5 * T_cmd + (N_cmd / B);
    
    %% 2. RATE LIMITING (Actuator Slew Rate)
    max_step = rate_max * dt;
    T_L_rate = min(max(T_L_des, T_L_prev - max_step), T_L_prev + max_step);
    T_R_rate = min(max(T_R_des, T_R_prev - max_step), T_R_prev + max_step);
    
    %% 3. THRUST SATURATION (Physical Limits)
    T_L_act = min(max(T_L_rate, T_min_ind), T_max_ind);
    T_R_act = min(max(T_R_rate, T_min_ind), T_max_ind);
    
    %% 4. RESULTING CONTROL FORCES ON HULL
    T_act = T_L_act + T_R_act;
    N_act = (T_R_act - T_L_act) * (B / 2);
    tau_ctrl = [T_act; 0.0; N_act];
    
    %% 5. PROPULSION ELECTRICAL POWER CONSUMPTION
    % Electromechanical mechanical output + I^2*R coil loss + static bollard loss
    u_eff = max(abs(u), 0.1);
    P_L = (abs(T_L_act) * u_eff) / eta_motor + k_loss * (T_L_act^2);
    P_R = (abs(T_R_act) * u_eff) / eta_motor + k_loss * (T_R_act^2);
    
    P_prop = P_L + P_R;
end
