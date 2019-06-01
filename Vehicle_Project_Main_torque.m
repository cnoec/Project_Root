% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

path        =           pwd;
addpath('Functions');
addpath('Model');
addpath('Mat_Data\');
addpath('unc_optimization');

%% track generation and waypoints positioning

% choose which track to plot:
% track_number = 1 -> CIRCLE TRACK
% track_number = 2 -> OVAL TRACK
% track_number = 3 -> RANDOM TRACK

track_number = 2;
[~,outerBoundary,innerBoundary,N,x0,y0] = track_generation(track_number);

n_wp = 30;

[ waypoints ] = waypoints_selector(innerBoundary,outerBoundary, n_wp,N);

%% parameters initialization and setting of initial state

m = (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m = -1/m;

X       =       x0;         % inertial X position (m)
Y       =       y0;         % inertial Y position (m)
Ux      =       20;         % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       atan(m);    % yaw angle (rad)

if (track_number == 1) 
    
    psi = psi + pi; 

end

r       =       0;          % yaw rate (rad/s)
xi0     =       [X Y Ux beta psi r]';

plot(X,Y,'*r');

%% Optimization

boundary_number     =       1;
tau                 =       0;
d                   =       0;
Ts                  =       1e-1; 
T_end               =       25;
n_iterations        =       T_end/Ts;

u                                   =       ones(2*n_iterations,1);
u(1:n_iterations)                   =       100;
u(n_iterations+1:2*n_iterations)    =       3*pi/180;

% for i = 1:n_iterations
%     
%     u(i)                    =       100*rand();
%     u(n_iterations+i)       =       3*pi/180*rand();
%     
% end

% u_d                                 =       ones(n_iterations,1)*3*pi/180;

% load('u_opt_20190528.mat');
% 
% u_d = u_opt();
% clear u_opt

% u_T                                 =       ones(n_iterations,1)*100;

[xi, t_vec, end_check]              =       trajectory_generation(u, xi0, T_end, Ts);

n_states                            =       length(xi);

tic
% [u_opt,dist_opt,n_iter,~,seq]       = myfminunc(@(u)(deltasum(u,xi0, T_end, Ts, waypoints, n_wp)...
%                                     ),u,myoptimalset);
[u_opt,exitflag, debug] = uncons_NLP_opt(@(u)deltasum(u,xi0, T_end, Ts, waypoints, n_wp),u,unc_optimalset)
toc
                            
[xi, ~, ~]    = trajectory_generation(u_opt, xi0, T_end, Ts);

%% optimal trajectory plot

figure
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal

hold on

for i=1:(n_states-1)
   plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'.r');
end

%% whole sequence plote

for i = 1:size(debug.seq,2)
    figure
    plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
        outerBoundary(:,2),'black'),grid on
    axis equal
    hold on
    u_check                     =       seq(:,i);
%     u = [u_T; u_check];
    [xi_1, ~, ~]                =       trajectory_generation(u_check, xi0, T_end, Ts);
    plot(xi_1(1,:), xi_1(2,:),'.');grid;
    title(i);
    pause(0.5)
%     close
end

%
% dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% dist = zeros(n_wp,1);
% min_dist_point = zeros(n_wp,2);
% min_index = 1;
 
% F   = fopen('prova.txt','w');
% fprintf(F,'Gradient \n');
% fprintf(F,'%f           %f \n',y,grad);

%%
