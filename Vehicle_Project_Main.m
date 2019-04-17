% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

%% track generation and waypoints positioning

[track,innerBoundary,outerBoundary,x0,y0,N] = track_generation();

%% parameters initialization and setting of initial state
run('Parameters.m');

X       =       x0;         % inertial X position (m)
Y       =       y0;         % inertial Y position (m)
Ux      =       4;          % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       0;          % yaw angle (rad)
r       =       0;          % yaw rate (rad/s)
xi0     =       [X Y Ux beta psi r]';

%% Step amplitude and input vector
Td_step             =       80
delta_step          =       50
u_output            =       [Td_step;delta_step];

%% trajectory optimizer


%% simulation

n_iterations    =       N-1;
tau             =       0;
d               =       0;

for i = 1:n_iterations
  % interpolator
  if i ~= 1
    xy  = [mean([innerBoundary(i+1,1,1) outerBoundary(N-i,1,1)]') mean([innerBoundary(i+1,2,1) outerBoundary(N-i,2,1)]')]'
    line = [xi0(1),xy(1);xi0(2),xy(2)];
    d = pdist(line,'euclidean')
    u_output(2) = real(asin(d))
  end
  % 
  
  [xi_dot,Forces] = Vehicle_Model_Function(tau,xi0,u_output,d,theta);
  % random xy coordinates generation - must to be replaced with the real
  % update of the target position
  x_target   = xi0(1) + cos(xi_dot(4))
  y_target   = xi0(2) + sin(xi_dot(4))
  xy  = [x_target y_target]';
  % check if the target position is feasible
  inside = track_constraints(xy(1),xy(2),N,innerBoundary,outerBoundary)
  % if the target position is feasible, update of the state with the new
  % position. else exit the for cycle, because the trajectory is not
  % feasible.
  if inside == 1
      plot([x_target xi0(1)],[y_target xi0(2)])
      pause(.001)
      xi0(1) = x_target;
      xi0(2) = y_target;
      xi0(3) = xi_dot(4);
  else
      return
  end
  
end


