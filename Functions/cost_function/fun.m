function v = fun(u, xi0, T_end, Ts, waypoints, n_wp ,inner_boundary,outer_boundary,road_width)

Ts_sim      =   1e-2;
n_states    =   T_end/Ts_sim;

[F,~,xi]    =   cost_function_gamma(u,xi0,T_end,Ts,waypoints,n_wp,7,1e6);

g           =     (xi(1,end)-xi(1,1));
g           =      g/1e4;

% h = track_constraint(xi,inner_boundary,outer_boundary,length(xi),road_width);

h_soft      =   [-(xi(2,end)-xi(2,1))+road_width/2;
                  (xi(2,end)-xi(2,1))+road_width/2;
                   xi(3,1)];
               
eps         =   0.05;

% h_track     =   is_it_in_track(xi,n_states,eps);

%%

h           =       [h_soft; 
                    track_constraint(xi,inner_boundary,outer_boundary,n_states,road_width)];

% h           =       h_soft;

% h           =       track_constraint(xi,inner_boundary,outer_boundary,n_states,road_width);

h           =       h/1e5;

v           =       [F;
                     g;
                     h];

end
