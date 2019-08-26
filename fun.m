function v = fun(u, xi0, T_end, Ts, waypoints, n_wp ,inner_boundary,outer_boundary,road_width)

u(1:T_end/Ts) = u(1:T_end/Ts)*2000;

[xi, ~, ~] = trajectory_generation(u, xi0, T_end, Ts);

n_states = length(xi);

dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% sum_Delta = sum(dist)/1e5;

state_diff = [xi(1,end) - xi(1,1);
              xi(2,end) - xi(2,1)];
          
          
F = sum(dist)'*sum(dist)/1e6;

g = state_diff;

% g = [];
% 
% h = track_constraint(xi,inner_boundary,outer_boundary,length(xi),road_width);

h = [];


v = [F;
     g;
     h];

end

