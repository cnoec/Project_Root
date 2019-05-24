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
Ux      =       10;         % body x velocity (m/s)
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

delta = ones(n_iterations,1)*3*pi/180;
Torque = 100*ones(n_iterations,1);

u_ini  = [Torque;delta];

%% initial guess

xi_ini = trajectory_generation(u_ini,xi0,T_end,Ts);
% y = unc_MINFUNC_gamma(u_ini, xi0, T_end, Ts, waypoints,0.2)

figure('Name','Initial Guess','NumberTitle', 'off')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal
hold on

% plot of finish line
line([innerBoundary(1,1,1) outerBoundary(1,1,1)],...
     [innerBoundary(1,2,1) outerBoundary(1,2,1)],'color','y','linewidth', 7) 

plot(waypoints(:,1), waypoints(:,2),'*b')

plot(xi_ini(1,:), xi_ini(2,:),'.g')
hold off

%% minimization

u_ini = [Torque; seq(:, end)]
% u_ini(1:n_iterations) = u_ini(1:n_iterations) + 100;
[u_opt,dist_opt,n_iter,~,sequenc] = myfminunc(@(u_opt)(unc_MINFUNC_gamma(u_opt, xi0, T_end, Ts, waypoints,0.01)),u_ini,myoptimalset);

 % AGGIUNGERE CODEGEN
 
%% plotting

print_vec = zeros(n_iter+1);

print_vec(1) = 1;
print_vec(10) = 1;
print_vec(20) = 1;
print_vec(30) = 1;
print_vec(40) = 1;


figure('Name', 'sequenceuence', 'NumberTitle', 'off' )
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal
hold on

% plot of finish line

line([innerBoundary(1,1,1) outerBoundary(1,1,1)],...
     [innerBoundary(1,2,1) outerBoundary(1,2,1)],'color','y','linewidth', 7) 
axis equal
hold on

plot(waypoints(:,1), waypoints(:,2),'*b')
hold on
%%
for i=1:min(size(sequenc))

%     u_check = [u_T; seq(:,i)];
    u_check = sequenc(:,i)
    [xi_1, ~, ~] = trajectory_generation(u_check, xi0, T_end, Ts);

    plot(xi_1(1,:), xi_1(2,:),'.');
    pause;

end
    