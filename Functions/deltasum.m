function [sum_Delta] = deltasum(u, xi0, T_end, Ts, waypoints, n_wp)

[xi, ~, ~] = trajectory_generation(u, xi0, T_end, Ts);

n_states = length(xi);

dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

sum_Delta = sum(dist);

end

