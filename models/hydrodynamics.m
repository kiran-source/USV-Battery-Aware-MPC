function [M, C, D, tau_D] = hydrodynamics(nu, nu_r, p_usv)
    % HYDRODYNAMICS Computes 3-DOF Marine Hydrodynamic Matrices and Forces
    % References: Fossen (2011), "Handbook of Marine Craft Hydrodynamics and Motion Control"
    %
    % Inputs:
    %   nu   = [u; v; r]     - USV body-fixed velocity vector
    %   nu_r = [u_r; v_r; r] - USV relative velocity vector (relative to water current)
    %   p_usv                - USV parameters struct
    %
    % Outputs:
    %   M     - Total 3x3 inertia matrix (Rigid-Body + Added Mass)
    %   C     - Total 3x3 Coriolis & Centripetal matrix (C_RB + C_A)
    %   D     - Total 3x3 Hydrodynamic Damping matrix (Linear + Quadratic)
    %   tau_D - Damping force vector [N; N; N*m]
    
    u   = nu(1);
    v   = nu(2);
    r   = nu(3);
    
    ur  = nu_r(1);
    vr  = nu_r(2);
    
    m   = p_usv.m;
    Iz  = p_usv.Iz;
    xg  = p_usv.xg;
    
    X_udot = p_usv.X_udot;
    Y_vdot = p_usv.Y_vdot;
    Y_rdot = p_usv.Y_rdot;
    N_vdot = p_usv.N_vdot;
    N_rdot = p_usv.N_rdot;
    
    %% 1. INERTIA MATRICES (Rigid Body + Added Mass)
    % Rigid Body Mass Matrix
    M_RB = [
        m,          0,          0;
        0,          m,          m * xg;
        0,          m * xg,     Iz
    ];
    
    % Hydrodynamic Added Mass Matrix
    M_A = -[
        X_udot,     0,          0;
        0,          Y_vdot,     Y_rdot;
        0,          N_vdot,     N_rdot
    ];
    
    M = M_RB + M_A;
    
    %% 2. CORIOLIS & CENTRIPETAL MATRICES
    % Rigid-body Coriolis matrix
    C_RB = [
        0,                  0,                  -m * (xg * r + v);
        0,                  0,                   m * u;
        m * (xg * r + v),  -m * u,               0
    ];
    
    % Added-mass Coriolis matrix (expressed in relative velocity)
    a1 = -X_udot * ur;
    a2 = -Y_vdot * vr - Y_rdot * r;
    
    C_A = [
        0,      0,      -a2;
        0,      0,       a1;
        a2,    -a1,      0
    ];
    
    C = C_RB + C_A;
    
    %% 3. HYDRODYNAMIC DAMPING (Linear + Nonlinear / Cross-flow)
    % Damping coefficients (defined negative in parameter file)
    d11 = -(p_usv.Xu + p_usv.Xuu * abs(ur));
    d22 = -(p_usv.Yv + p_usv.Yvv * abs(vr));
    d23 = -(p_usv.Yr + p_usv.Yrr * abs(r));
    d32 = -(p_usv.Nv + p_usv.Nvv * abs(vr));
    d33 = -(p_usv.Nr + p_usv.Nrr * abs(r));
    
    D = [
        d11,    0,      0;
        0,      d22,    d23;
        0,      d32,    d33
    ];
    
    % Hydrodynamic drag force opposes relative velocity
    tau_D = D * nu_r;
end
