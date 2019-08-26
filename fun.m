function v = fun(u, xi0, T_end, Ts, waypoints, n_wp ,inner_boundary,outer_boundary,road_width)

u(1:T_end/Ts) = u(1:T_end/Ts)*2000;

[xi, ~, ~] = trajectory_generation(u, xi0, T_end, Ts);

n_states = length(xi);

dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% sum_Delta = sum(dist)/1e5;

state_diff = [xi(1,end) - xi(1,1);
              xi(2,end) - xi(2,1)];
          
          
F = sum(dist)'*sum(dist)/10 + state_diff'*state_diff*10;

F = F/1e10;
          
% Q = eye(length(state_diff))*1e5;

% F = state_diff'*state_diff/1e9 ;

% F = F/1e7;

% g1 = xi(1,end) - xi(1,1);
% g2 = xi(2,end) - xi(2,1);

% g = [g1;
%      g2];
%  
% g = g/1e7;

g = state_diff/1e2;

% g = [];

% h = track_constraint(xi,inner_boundary,outer_boundary,n_states,road_width);

% h = (-state_diff+5)/1e2;

h = [];


v = [F;
     g;
     h];

end

