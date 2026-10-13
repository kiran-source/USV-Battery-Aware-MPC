function [SOC_next, E_used_next, P_batt, V_term, I_batt] = battery_model(P_prop, is_charging, SOC_cur, E_used_cur, dt, p_bat)
    % BATTERY_MODEL Simulates USV Battery Energy, SOC, and Terminal Circuit Dynamics
    %
    % Inputs:
    %   P_prop      - Total electrical propulsion power [W]
    %   is_charging - Boolean flag (true if docked at charging station)
    %   SOC_cur     - Current State of Charge [%]
    %   E_used_cur  - Current cumulative energy used [Joules]
    %   dt          - Integration timestep [s]
    %   p_bat       - Battery parameter struct
    %
    % Outputs:
    %   SOC_next    - Updated State of Charge [%]
    %   E_used_next - Updated cumulative energy [Joules]
    %   P_batt      - Net power flow from/to battery (+ = discharging, - = charging) [W]
    %   V_term      - Terminal voltage [V]
    %   I_batt      - Battery current (+ = discharge, - = charge) [A]
    
    P_hotel = p_bat.P_hotel;
    E_cap   = p_bat.capacity_J;
    
    %% 1. NET POWER CALCULATION
    if is_charging
        % Net power entering battery during fast charging
        P_batt = P_hotel - (p_bat.P_charge * p_bat.charge_efficiency);
    else
        % Normal mission discharge
        P_batt = P_prop + P_hotel;
    end
    
    %% 2. ENERGY & SOC INTEGRATION (Coulomb Counting)
    dE = P_batt * dt;
    E_used_next = E_used_cur + dE;
    
    % SOC percentage update
    dSOC = -(P_batt * dt / E_cap) * 100.0;
    SOC_next = min(max(SOC_cur + dSOC, 0.0), p_bat.soc_max_usable);
    
    %% 3. OPTIONAL EQUIVALENT CIRCUIT MODEL (Level B Voltage / Current)
    % Open-circuit voltage polynomial approximation for 24V LiFePO4 pack
    soc_frac = SOC_next / 100.0;
    Voc = p_bat.circuit.V_min + (p_bat.circuit.V_max - p_bat.circuit.V_min) * (0.8 * soc_frac + 0.2 * (soc_frac^0.5));
    
    % Current I from P = V*I - I^2*R -> I ≈ P / Voc
    R_int = p_bat.circuit.R_int;
    if abs(P_batt) > 0.1
        % Quadratic formula for real circuit: P = Voc*I - I^2*R_int
        disc = max(Voc^2 - 4 * R_int * P_batt, 0.0);
        I_batt = (Voc - sqrt(disc)) / (2 * R_int);
    else
        I_batt = 0.0;
    end
    
    V_term = Voc - I_batt * R_int;
end
