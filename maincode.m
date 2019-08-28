clear all
close all
clc

path                    =   pwd;
addpath('Functions');
addpath('Model');
addpath('Mat_Data\');
addpath('Functions\unc_optimization');
addpath('Functions\con_optimization');
addpath('Functions\track');
addpath('Functions\cost_function');

%%

Ts = 0.1;
T_end = 25;

track_number        =   2;
[~,outerBoundary,innerBoundary,N,x0,y0] = track_generation(track_number);

m = (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m = -1/m;

X       =       x0;         % inertial X position (m)
Y       =       y0;         % inertial Y position (m)
Ux      =       20;         % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       atan(m);    % yaw angle (rad)
r       =       0;          % yaw rate (rad/s)
xi_0     =       [X Y Ux beta psi r]';

t_vec = 0:Ts:T_end;
t_vec = t_vec';
N = length(t_vec); 
Ts_1ms = 1e-3;

u = [20;
     50;
     50;
     50;
    0.61/15*ones(N,1);];
[xi, ~, end_check] = trajectory_generation_sim(u, xi_0, T_end, Ts);

%%
figure
plot(xi(:,1), xi(:,2));






