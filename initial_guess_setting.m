%% Track setting

% choose which track to plot:
% track_number = 1 -> CIRCLE TRACK
% track_number = 2 -> OVAL TRACK
% track_number = 3 -> RANDOM TRACK

track_number        =   2;
[~,outerBoundary,innerBoundary,N,x0,y0] = track_generation(track_number);

n_wp                =   30;
[ waypoints ] = waypoints_selector(innerBoundary,outerBoundary, n_wp,N);

%% Initial Guess Setting

m                   =   (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m                   =   -1/m;

X                   =   x0;         % inertial X position (m)
Y                   =   y0;         % inertial Y position (m)
Ux                  =   20;         % body x velocity (m/s)
beta                =   0;          % sideslip angle (rad)
psi                 =   atan(m);    % yaw angle (rad)

if (track_number == 1) 
    
    psi             =   psi + pi; 

end

r                   =       0;          % yaw rate (rad/s)
xi0                 =       [X Y Ux beta psi r]';

T_end               =       25;
Ts                  =       1e-1;
n_iterations        =       T_end/Ts;


u_T                          =      ones(n_iterations,1)*100;

u_0                          =      [u_T;u_0];

[xi, t_vec, end_check]       =      trajectory_generation(u_0,xi0,T_end,Ts);

n_states                     =      length(xi);

%%