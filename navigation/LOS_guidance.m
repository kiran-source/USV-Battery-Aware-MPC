function [psi_ref, e_y, s_track, alpha_k] = LOS_guidance(pos_xy, pt_start, pt_end, Delta_LOS)
    % LOS_GUIDANCE Line-of-Sight Steering Guidance Law for Straight-Line Path Segments
    %
    % Inputs:
    %   pos_xy    - [x, y] current USV position in NED [m]
    %   pt_start  - [x_k, y_k] start of active path segment [m]
    %   pt_end    - [x_{k+1}, y_{k+1}] end of active path segment [m]
    %   Delta_LOS - Lookahead distance [m] (default = 25.0 m)
    %
    % Outputs:
    %   psi_ref   - Desired Line-of-Sight heading angle [rad]
    %   e_y       - Cross-track error (+ = vessel is to the right of path) [m]
    %   s_track   - Along-track progression distance along segment [m]
    %   alpha_k   - Path azimuth angle [rad]
    
    if nargin < 4, Delta_LOS = 25.0; end
    
    x = pos_xy(1); y = pos_xy(2);
    x_k = pt_start(1); y_k = pt_start(2);
    x_kp1 = pt_end(1); y_kp1 = pt_end(2);
    
    dx = x_kp1 - x_k;
    dy = y_kp1 - y_k;
    
    % Path segment orientation angle
    alpha_k = atan2(dy, dx);
    
    % Relative position vector from segment origin
    px = x - x_k;
    py = y - y_k;
    
    % Cross-track error (perpendicular distance to path line)
    e_y = -sin(alpha_k) * px + cos(alpha_k) * py;
    
    % Along-track distance (projection along path direction)
    s_track = cos(alpha_k) * px + sin(alpha_k) * py;
    
    % LOS guidance formula (Fossen 2011)
    psi_los_offset = atan2(-e_y, Delta_LOS);
    psi_ref = wrapToPi(alpha_k + psi_los_offset);
end
