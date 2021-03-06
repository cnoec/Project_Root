function [sum_Delta] = deltasum(u, xi0, T_end, Ts, waypoints, n_wp)

[xi, ~, ~, ~] = trajectory_generation_cc(u, xi0, T_end, 1e-1, 1e-2);

n_states = length(xi);

dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% sum_Delta = sum(dist)/1e5;

sum_Delta = dist'*dist/1e7;

end

