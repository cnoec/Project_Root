% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

path        =           pwd;
addpath('Functions');
addpath('Model');
addpath('Mat_Data\');
addpath('Functions\unc_optimization');
addpath('Functions\con_optimization');
addpath('Functions\track');
addpath('Functions\cost_function');


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


T_end   =       25;
Ts_steer      =       1e-1;
Ts_sim = 1e-2;

u_0     =       [20;
                 20;
                 3/15*ones(T_end/Ts_steer,1)];
             
%% cc

global num_dyn den_dyn num_int


s = tf('s');

CC_int                                  =   90/s;
CC_int_d                                =   c2d(CC_int, Ts_sim, 'tustin');
[num_int, den_int]                      =   tfdata(CC_int_d, 'v');

CC_dyn                                  =   920/(s/10 + 1);
CC_dyn_d                                =   c2d(CC_dyn, Ts_sim, 'tustin');
[num_dyn, den_dyn]                      =   tfdata(CC_dyn_d, 'v');

%% Optimization


u_opt = [20; 20; u_opt];
u_opt(62:end) = 0.5/15;
u_0 = u_opt;

%%
tic
[u_opt,~,debug] = uncons_NLP_opt(@(u)(cost_function_gamma(u,xi0,T_end,Ts_steer,waypoints,n_wp,20,1e6)),u_0,unc_optimalset);
unc_time = toc
                           
%% optimal trajectory plot

[xi, t_vec, ~ ,torque]    = trajectory_generation_cc(u_0, xi0, T_end, 0.1,1e-2);

figure('Name', 'Optimal Trajectory', 'NumberTitle', 'off')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal
hold on
plot(xi(1,:),xi(2,:),'.g')
hold off

figure('Name', 'Torque', 'NumberTitle', 'off')
plot(t_vec(1:end-1),torque);grid;title('Torque/s [Nm/s]');

figure('Name', 'Optimal Trajectory', 'NumberTitle', 'off')
plot(t_vec,xi(3,:));grid;title('Speed  [Nm/s]');


%% sequence

sequence = debug.seq;
figure('Name', 'Sequence', 'NumberTitle', 'off')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal
hold on
count = size(sequence,2);

for j = 1:count
    figure('Name', 'Sequence', 'NumberTitle', 'off')
    plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
    axis equal
    [xi_s,~,~,~]    =    trajectory_generation_cc(sequence(:,j), xi0, T_end, 0.1,1e-2);
    plot(xi_s(1,:),xi_s(2,:));grid;
    j
    count
    
    pause
    
end

hold off
%%

% test cost function
% caricato da Mat_Data
u_opt(68:end) = 0.5/15;

u_0 = [20; 15; u_opt];

%%

[xi, t_vec, exitflag, torque] =  trajectory_generation_cc(u_0, xi0, T_end, Ts_steer, Ts_sim);

[cost, point] = cost_function_gamma(u_0, xi0, T_end, Ts_steer, waypoints, n_wp, 5, 1e7);
disp('ok')

%%

figure('Name', 'Optimal Trajectory', 'NumberTitle', 'off')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal
hold on
plot(xi(1,:),xi(2,:))
hold on
for ind = 1:n_wp
    plot(xi(1,point(ind)),xi(2,point(ind)), '*r');
    hold on
    txt = {ind};
    text(xi(1,point(ind)),xi(2,point(ind)),txt);
end
hold off






