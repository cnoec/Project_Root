function v = fun(u, xi0, T_end, Ts, waypoints, n_wp)

u(1:T_end/Ts) = u(1:T_end/Ts)*2000;

[xi, ~, ~] = trajectory_generation(u, xi0, T_end, Ts);

n_states = length(xi);

dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% sum_Delta = sum(dist)/1e5;

F = dist'*dist/1e7;

g1 = xi(1,end) - xi(1,1);
g2 = xi(2,end) - xi(2,1);

g = [g1;
     g2];
 
g = g/1e5;

h = [];

v = [F;
     g;
     h];

end

