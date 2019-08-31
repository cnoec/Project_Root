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
Ts_sim              =       1e-2;
n_iterations        =       T_end/Ts;

%%