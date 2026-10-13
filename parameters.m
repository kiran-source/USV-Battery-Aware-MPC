function params = parameters()
    % PARAMETERS Initializes all parameters for USV Battery-Aware MPC Simulation
    % First-Evaluation Prototype
    
    %% USV Physical Dynamics (1-DOF Surge)
    params.usv.m  = 120.0;          % Boat mass [kg]
    params.usv.Cd = 25.0;           % Quadratic drag coefficient [N*s^2/m^2]
    params.usv.T_max = 200.0;       % Maximum forward thrust [N]
    params.usv.T_min = 0.0;         % Minimum thrust [N] (no reverse required for prototype)
    params.usv.u_max = 3.5;         % Maximum operating speed [m/s]
    params.usv.u_init = 0.0;        % Initial speed [m/s]
    params.usv.x_init = 0.0;        % Initial position [m]

    %% Mission Configuration
    params.mission.x_start = 0.0;      % Start distance [m]
    params.mission.x_dest  = 1000.0;   % Destination distance [m]
    params.mission.arrival_tol = 5.0;  % Arrival tolerance [m]
    params.mission.T_sim = 1100.0;     % Total simulation duration [s]
    params.mission.dt = 0.1;           % Fundamental simulation timestep [s]

    %% Battery & Power Model
    % Nominal capacity set so that 1000m mission at 2.0 m/s consumes ~70-75% SOC
    % Steady-state thrust at 2 m/s = Cd * u^2 = 25 * 4 = 100 N
    % Power at 2 m/s = T * u = 200 W (+ 10W aux = 210 W)
    % Travel time for 1000 m @ 2 m/s = 500 s -> Energy = 210 W * 500 s = 105,000 J
    % With capacity = 150,000 J (41.67 Wh), 105,000 J is 70% of total capacity.
    params.battery.capacity_J = 150000.0; % Battery total capacity [Joules]
    params.battery.P_aux = 10.0;          % Hotel/avionics auxiliary power [W]
    params.battery.initial_soc = 80.0;    % Default initial SOC [%]
    params.environment.current_speed = 0.2;
    
    params.environment.current_dir = 0.0;
    params.usv.psi_init = 0.0;
    %% Battery-Aware Speed Scheduling Rules
    % High SOC (> 60%): Normal cruise
    params.battery_aware.soc_threshold_high = 60.0; % [%]
    params.battery_aware.speed_high = 2.0;          % [m/s]

    % Medium SOC (30% - 60%): Energy saving
    params.battery_aware.soc_threshold_low = 30.0;  % [%]
    params.battery_aware.speed_med = 1.5;           % [m/s]

    % Low SOC (< 30%): Low power / range extension
    params.battery_aware.speed_low = 1.0;           % [m/s]

    %% PID Controller Tuning
    params.pid.Kp = 120.0;
    params.pid.Ki = 15.0;
    params.pid.Kd = 18.0;
    params.pid.N  = 50.0;           % Filter coefficient for derivative term
    params.pid.anti_windup = true;

    %% MPC Controller Configuration
    params.mpc.Ts = 0.5;            % MPC sampling time [s]
    params.mpc.Np = 15;             % Prediction horizon [steps] (7.5 seconds)
    params.mpc.Nc = 4;              % Control horizon [steps] (2.0 seconds)
    
    % MPC Cost Function Weights:
    % J = sum( w1*(u - u_ref)^2 + w2*Power + w3*(delta_T)^2 )
    params.mpc.w_speed = 100.0;     % Speed tracking weight (w1)
    params.mpc.w_power = 0.04;      % Energy/power penalty weight (w2)
    params.mpc.w_rate  = 0.02;      % Thrust slew rate penalty weight (w3)

    %% Feasibility Estimator Parameters
    params.feasibility.min_speed_eval = 0.1; % [m/s] avoid division by zero
end
