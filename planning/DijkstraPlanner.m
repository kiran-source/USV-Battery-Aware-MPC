function [node_path, total_dist] = DijkstraPlanner(start_node_id, goal_node_id, nodes, adj_matrix)
    % DIJKSTRAPLANNER Topological Graph-Level Shortest Path Search
    %
    % Inputs:
    %   start_node_id - ID of starting location (e.g. Base = 1)
    %   goal_node_id  - ID of goal location (e.g. Dest = 6)
    %   nodes         - Node coordinates struct or matrix
    %   adj_matrix    - Adjacency matrix of edge distances / costs
    
    N_nodes = size(adj_matrix, 1);
    dist = inf(N_nodes, 1);
    prev = zeros(N_nodes, 1);
    unvisited = 1:N_nodes;
    
    dist(start_node_id) = 0;
    
    while ~isempty(unvisited)
        [~, min_idx] = min(dist(unvisited));
        u = unvisited(min_idx);
        
        if u == goal_node_id || isinf(dist(u))
            break;
        end
        
        unvisited(min_idx) = [];
        
        neighbors = find(adj_matrix(u, :) > 0);
        for i = 1:length(neighbors)
            v = neighbors(i);
            alt = dist(u) + adj_matrix(u, v);
            if alt < dist(v)
                dist(v) = alt;
                prev(v) = u;
            end
        end
    end
    
    % Path extraction
    node_path = [];
    curr = goal_node_id;
    while curr > 0
        node_path = [curr, node_path]; %#ok<AGROW>
        curr = prev(curr);
    end
    
    if isempty(node_path) || node_path(1) ~= start_node_id
        node_path = [start_node_id, goal_node_id];
        total_dist = norm([nodes(goal_node_id).x - nodes(start_node_id).x, ...
                           nodes(goal_node_id).y - nodes(start_node_id).y]);
    else
        total_dist = dist(goal_node_id);
    end
end
