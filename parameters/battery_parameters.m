function p_bat = battery_parameters()
    % BATTERY_PARAMETERS Configuration for USV LiFePO4 / Li-Ion Battery Pack
    
    % --- Energy-Based Parameters ---
    p_bat.capacity_Wh     = 1200.0; % Nominal usable battery capacity [Wh] (24V 50Ah pack)
    p_bat.capacity_J      = p_bat.capacity_Wh * 3600.0; % 4.32 MJ [Joules]
    
    p_bat.initial_soc_pct = 80.0;   % Default mission initial SOC [%]
    p_bat.soc_min_pct     = 5.0;    % Hardware cutoff minimum SOC [%]
    p_bat.soc_reserve_pct = 20.0;   % Mandatory emergency reserve threshold [%]
    p_bat.soc_economy_pct = 50.0;   % Economy mode trigger threshold [%]
    p_bat.soc_max_usable  = 100.0;  % Maximum allowable SOC [%]
    
    % --- Continuous Electrical Loads ---
    p_bat.P_hotel         = 35.0;   % Avionics, sensors, microcontrollers, GPS, telemetry [W]
    
    % --- Optional Voltage/Current Circuit Model (Level B) ---
    p_bat.circuit.V_nominal = 24.0;  % Nominal pack voltage [V]
    p_bat.circuit.V_max     = 29.2;  % Full charge voltage [V]
    p_bat.circuit.V_min     = 20.0;  % Cutoff voltage [V]
    p_bat.circuit.R_int     = 0.045; % Internal pack resistance [Ohms]
    
    % --- Charging Station Model ---
    p_bat.P_charge          = 600.0; % Fast charge dock power rate [W] (25A at 24V)
    p_bat.charge_efficiency = 0.92;  % Dock charger efficiency
end
