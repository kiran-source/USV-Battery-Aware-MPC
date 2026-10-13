function [t_out, x_out, u_out, SOC_out, P_out] = USV_1DOF_reference(p, T_cmd_vec, t_span, initial_soc)
    % USV_1DOF_REFERENCE Benchmark 1-DOF Longitudinal Surge Reference Model
    % Governed by:
    %   m * du/dt = T - (c1*u + c2*u^2)
    %   dx/dt = u
    %   P = T*u + P_aux
    %   SOC(t) = SOC_0 - (integral(P dt) / E_cap) * 100
    
    if nargin < 4, initial_soc = 80.0; end
    if nargin < 3, t_span = [0 600]; end
    
    m = p.usv.m;
    c1 = abs(p.usv.Xu);
    c2 = abs(p.usv.Xuu);
    P_aux = p.battery.P_hotel;
    E_cap = p.battery.capacity_J;
    
    dt = 0.1;
    t_out = (t_span(1):dt:t_span(2))';
    N = length(t_out);
    
    x_out = zeros(N, 1);
    u_out = zeros(N, 1);
    SOC_out = zeros(N, 1);
    P_out = zeros(N, 1);
    
    x = 0; u = 0; SOC = initial_soc;
    
    for k = 1:N
        if isscalar(T_cmd_vec)
            T = T_cmd_vec;
        else
            T = T_cmd_vec(min(k, length(T_cmd_vec)));
        end
        T = min(max(T, 0), p.usv.prop.T_max_tot);
        
        % Drag force
        D = c1*u + c2*(u^2);
        
        % Acceleration & Integration (RK4 / Euler)
        du = (T - D) / m;
        u = max(u + du * dt, 0);
        x = x + u * dt;
        
        % Power & SOC
        P = T * u + P_aux;
        SOC = max(SOC - (P * dt / E_cap) * 100, 0);
        
        t_out(k) = (k-1)*dt;
        x_out(k) = x;
        u_out(k) = u;
        SOC_out(k) = SOC;
        P_out(k) = P;
    end
end
