function [wp_idx_next, pt_prev, pt_next, dist_to_wp, is_dest_reached] = waypoint_manager(pos_xy, waypoints, wp_idx_cur, p_mis)
    % WAYPOINT_MANAGER Tracks and Transitions Waypoint Sequence along Path
    %
    % Inputs:
    %   pos_xy        - [x, y] current USV position [m]
    %   waypoints     - [N x 2] matrix of path waypoints
    %   wp_idx_cur    - Current active waypoint target index (1 to N)
    %   p_mis         - Mission parameter struct
    %
    % Outputs:
    %   wp_idx_next   - Updated active waypoint index
    %   pt_prev       - [x, y] segment start point
    %   pt_next       - [x, y] segment target point
    %   dist_to_wp    - Distance to active waypoint [m]
    %   is_dest_reached - Boolean flag
    
    N_wp = size(waypoints, 1);
    wp_idx_cur = max(min(wp_idx_cur, N_wp), 1);
    
    pt_next = waypoints(wp_idx_cur, :);
    if wp_idx_cur > 1
        pt_prev = waypoints(wp_idx_cur - 1, :);
    else
        pt_prev = pos_xy;
    end
    
    dist_to_wp = norm(pos_xy - pt_next);
    
    % Capture radius criteria
    if wp_idx_cur == N_wp
        R_cap = p_mis.guidance.R_capture_dest;
    else
        R_cap = p_mis.guidance.R_capture_wp;
    end
    
    % Switch to next waypoint if within capture circle
    if dist_to_wp <= R_cap && wp_idx_cur < N_wp
        wp_idx_next = wp_idx_cur + 1;
        pt_prev = pt_next;
        pt_next = waypoints(wp_idx_next, :);
        dist_to_wp = norm(pos_xy - pt_next);
    else
        wp_idx_next = wp_idx_cur;
    end
    
    is_dest_reached = (wp_idx_next == N_wp) && (dist_to_wp <= p_mis.guidance.R_capture_dest);
end
