% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

path        =           pwd;
addpath('Functions');
addpath('Model');

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
r       =       0;          % yaw rate (rad/s)
xi0     =       [X Y Ux beta psi r]';

plot(X,Y,'*r')

%% simulation

boundary_number     =       1;
tau                 =       0;
d                   =       0;
Ts                  =       1e-1; 
T_end               =       10;
n_iterations        =       T_end/Ts;

u = ones(2*n_iterations,1);
u(1:n_iterations) = 150;
u(n_iterations+1:2*n_iterations) = 5*pi/180;

[xi, t_vec, end_check] = trajectory_generation(u, xi0, T_end, Ts);

n_states = length(xi);

for i=1:(n_states-1)
   plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'*r');
end


% fai la prova con questi parametri, esce 76.5 ma dovrebbe essere circa 22
% target = [ -40; 34.74];
% dist = wp_to_trajectory_distance( target, xi(1:2,:)', 'only' )



