% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

path = pwd;
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

close all

%% parameters initialization and setting of initial state

m = (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m = -1/m;

X       =       x0;         % inertial X position (m)
Y       =       y0;         % inertial Y position (m)
Ux      =       20;         % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       atan(m);    % yaw angle (rad)
r       =       0;          % yaw rate (rad/s)

if (track_number == 1) 
    psi = psi + pi; 
end

xi0     =       [X Y Ux beta psi r]';

%% simulation parameters

tau                 =       0;
d                   =       0;
Ts                  =       1e-1; 
T_end               =       25;
n_iterations        =       T_end/Ts;

% delta_ini = ones(n_iterations,1)*3*pi/180;

Torque = 100*ones(n_iterations,1);

load sequence_fagiano
delta_ini = sequence_fagiano(:,1);          % INITIAL GUESS FOR NO TORQUE OPTIMIZATION

u_ini  = [Torque;delta_ini];                % INITIAL GUESS FOR COMPLETE OPTIMIZATION

%% initial guess

xi_ini = trajectory_generation(u_ini,xi0,T_end,Ts);
% y = unc_MINFUNC_gamma_noT(delta_ini, Torque, xi0, T_end, Ts, waypoints,0.1)

figure('Name','Initial Guess','NumberTitle', 'off')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal
hold on

line([innerBoundary(1,1,1) outerBoundary(1,1,1)],...
     [innerBoundary(1,2,1) outerBoundary(1,2,1)],'color','y','linewidth', 7) 

plot(waypoints(:,1), waypoints(:,2),'*b')

plot(xi_ini(1,:), xi_ini(2,:),'.g')
hold off

%% minimization with torque

u_ini = [Torque; delta_ini];
[u_opt_both,dist_opt_both,n_iter_both,exit_flag_both,sequence_both] = myfminunc(@(u_opt)(unc_MINFUNC_gamma(u_opt, xi0, T_end, Ts, waypoints,0.01)),u_ini,myoptimalset);

 % AGGIUNGERE CODEGEN
 
%% minimization without torque

[u_opt_delta,dist_opt_delta,n_iter_delta,~,sequence_delta] = myfminunc(@(u_opt)(unc_MINFUNC_gamma_noT(u_opt, Torque, xi0, T_end, Ts, waypoints,0.01,1)),delta_ini,myoptimalset);

 
%% plotting

figure('Name', 'Sequence', 'NumberTitle', 'off' )
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black')
hold on
line([innerBoundary(1,1,1) outerBoundary(1,1,1)],...
     [innerBoundary(1,2,1) outerBoundary(1,2,1)],'color','y','linewidth', 7) 
plot(waypoints(:,1), waypoints(:,2),'*b')
axis equal
grid on

%% plotting delta optimization

sequence_plot = sequence_delta;



for i=150:size(sequence_plot,2)
    i
    figure('Name', 'Sequence', 'NumberTitle', 'off' )
    plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black')

    hold on
    line([innerBoundary(1,1,1) outerBoundary(1,1,1)],...
     [innerBoundary(1,2,1) outerBoundary(1,2,1)],'color','y','linewidth', 7) 
    plot(waypoints(:,1), waypoints(:,2),'*b')
    axis equal
    grid on
    u_plot = [Torque; sequence_plot(:,i)];
    [xi_delta, ~, end_check] = trajectory_generation(u_plot, xi0, T_end, Ts);
    plot(xi_delta(1,:), xi_delta(2,:),'.');
    pause;
    close all
end



    