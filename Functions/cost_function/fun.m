function v = fun(u, xi0, T_end, Ts, waypoints, n_wp ,inner_boundary,outer_boundary,road_width)

% dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% sum_Delta = sum(dist)/1e5;

[F,~,xi] = cost_function_gamma(u,xi0,T_end,Ts,waypoints,n_wp,7,1e7);
          
% F = sum(dist)'*sum(dist)/1e6;


% g           = [xi(1,end) - xi(1,1);
%                xi(2,end) - xi(2,1)];
% g           = g/1e2;

g = (xi(1,end)-xi(1,1));
g= g/1e2;

% h = track_constraint(xi,inner_boundary,outer_boundary,length(xi),road_width);

h            = [-(xi(2,end)-xi(2,1))+road_width/2;
                 (xi(2,end)-xi(2,1))+road_width/2;
                  xi(3,1)];

h = h/1e4;
% h = [];


v = [F;
     g;
     h];

end
