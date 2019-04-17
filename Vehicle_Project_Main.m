% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

%% track generation and waypoints positioning

[track,innerBoundary,outerBoundary,x0,y0,N] =       track_generation();

% This function has to return two vectors containing the inner point and
% the outer one of each wayline.
[ inner_wl, outer_wl ]                      =       waylines_selector();

n_wl                                        =       length(inner_wl);

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
Td_step             =       80; % for now - random number
delta_step          =       50; % for now - random number
u_output            =       [Td_step;delta_step];

%% trajectory optimizer

[T_opt, delta_opt]  =       optimizer();

%% simulation

n_iterations    =       N-1;
tau             =       0;
d               =       0;

for i = 1:n_iterations
  
  % current wayline selection  
  
  current_wl    =    current_wl = current_wayline(inner_wl,outer_wl,boundary_number,innerBoundary,outerBoundary,n_wl,N)
  
  
  if (current_wl == n_wl)
  % If we are in correspondence of the last wayline we have to keep the
  % optimal values until we reach the end of the track, otherwise we'll
  % have computational problem.
      u_output          =           [T_opt(current_wl);delta_opt(current_wl)];
  
  else
      
      % interpolator

      d_interpolated    =       interpolator_bws( inner_wl(current_wl),outer_wl(current_wl),inner_wl(current_wl+1),outer_wl(current_wl+1),xi0(1),xi0(2));

      T_kp1             =       T_opt(current_wl) + (T_opt(current_wl+1) - T_opt(current_wl))*d_interpolated;
      delta_kp1         =       delta_opt(current_wl) + (delta_opt(current_wl+1) - delta_opt(current_wl))*d_interpolated;

      u_output          =       [T_kp1;delta_kp1];
  
  end
  
  % from the output of the interpolator to the new state
  [xi_dot,Forces] = Vehicle_Model_Function(tau,xi0,u_output,d,theta);
  
  % update of the target position
  x_target   = xi0(1) + xi_dot(3)*cos(xi_dot(4));
  y_target   = xi0(2) + xi_dot(3)*sin(xi_dot(4));
  
  % check if the target position is feasible
  inside = track_constraints(x_target,y_target,N,innerBoundary,outerBoundary);
  
  % if the target position is feasible, update of the state with the new
  % position. else exit the for cycle, because the trajectory is not
  % feasible.
  if inside == 1
      plot([x_target xi0(1)],[y_target xi0(2)]);
      pause(.001)
      xi0(1) = x_target;
      xi0(2) = y_target;
      xi0(3) = xi_dot(3);
      xi0(4) = xi_dot(4);
      xi0(5) = xi_dot(5);
      xi0(6) = xi_dot(6);
  else
      return
  end
  
end

