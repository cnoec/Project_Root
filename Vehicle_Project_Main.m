% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

path        =           pwd;
addpath('Functions');
addpath('Model');

%% track generation and waypoints positioning

[track,outerBoundary,innerBoundary,N] = track_generation();
n_wl=30;

aux1 = zeros(N,3);
aux2 = zeros(N,3);

for i=1:N
   if(i==N)
       aux1(i,:) = outerBoundary(1,:); 
       aux2(i,:) = innerBoundary(2,:);
   else
       aux1(i,:) = outerBoundary(N-i+1,:); 
       aux2(i,:) = innerBoundary(i+1,:);
   end
end
innerBoundary = aux2;
outerBoundary = aux1;
clear aux1 aux2 

figure
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal
hold on

% plot of finish line and initialization the starting position
line([innerBoundary(1,1,1) outerBoundary(1,1,1)],...
    [innerBoundary(1,2,1) outerBoundary(1,2,1)],'color','b','linewidth', 7) 
x   = [innerBoundary(1,1,1) outerBoundary(1,1,1)]';
y   = [innerBoundary(1,2,1) outerBoundary(1,2,1)]';
x0  = mean(x)+1;
y0  = mean(y);
plot(x0,y0,'*r')


% This function has to return two vectors containing the inner point and
% the outer one of each wayline.
[ inner_wl, outer_wl] = waylines_selector(innerBoundary,outerBoundary, n_wl);

% inner_wl(3,:) = [-61.13 -45.3 0];
% outer_wl(3,:) = [-62.58 -51.43 0];
% plot waylines

for i = 1:n_wl
    line([inner_wl(i,1,1) outer_wl(i,1,1)],[inner_wl(i,2,1) outer_wl(i,2,1)],...
        'color','y','linewidth', 5)
    txt = {i};
    text(inner_wl(i,1,1)+5,inner_wl(i,2,1)+2,txt)
end   

%% parameters initialization and setting of initial state
run('Parameters.m');

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
%initialization of the simulation, da ottimiz-zare
xi_sim(:,1)      =       xi0;

%% Step amplitude and input vector
Td_step             =       150; % for now - random number
delta_step          =       5; % for now - random number
u_output            =       [Td_step;delta_step];

%% trajectory optimizer

T_opt               =       [Tdmax;
                             Tdmax;
                             Tdmax/10;
                             Tdmax;
                             Tdmax/10;
                             Tdmax/10;];
                         
delta_opt           =       [0;
                             -pi/9;
                            -0.0001*pi;
                            -pi/2;
                             pi/9;
                             150;];

% [T_opt, delta_opt]  =       optimizer();

%% simulation

n_iterations        =       1000;
boundary_number     =       1;
tau                 =       0;
d                   =       0;
Ts                  =       1e-2;  

for i = 2:n_iterations %end of the iteration when we reach the final wl
  % current wayline selection  
  current_wl    =     current_wayline(inner_wl,outer_wl,boundary_number,...
                      innerBoundary,outerBoundary,n_wl,N)

%   if (current_wl == n_wl)
  % If we are in correspondence of the last wayline we have to keep the
  % optimal values until we reach the end of the track, otherwise we'll
  % have computational problem.
%       u_output          =           [T_opt(current_wl);delta_opt(current_wl)];
  
%   else
      
      % interpolator

      d_interpolated    =       interpolator_bws( inner_wl(current_wl,1:2),...
                                outer_wl(current_wl,1:2),inner_wl(current_wl+1,1:2),...
                                outer_wl(current_wl+1,1:2),xi_sim(1,i-1),xi_sim(2,i-1));

      T_kp1             =       T_opt(current_wl) + (T_opt(current_wl+1) ...
                                - T_opt(current_wl))*d_interpolated;
      delta_kp1         =       delta_opt(current_wl) + (delta_opt(current_wl+1)...
                                - delta_opt(current_wl))*d_interpolated;

      u_output          =       [T_kp1;delta_kp1];
  
%   end
  
  % from the output of the interpolator to the new state
  xi_sim(:,i) = xi_sim(:,i-1) + Ts*Vehicle_Model_Function(0, xi_sim(:,i-1),u_output, 0, theta);
  
  % check if the target position is feasible
  [inside,boundary_number] = track_constraints(xi_sim(1,i),xi_sim(2,i),N,innerBoundary,outerBoundary,boundary_number)
 
  % if the target position is feasible, update of the state with the new
  % position. else exit the for cycle, because the trajectory is not
  % feasible.
  
%   if inside == 1
     plot([xi_sim(1,i) xi_sim(1,i-1)],[xi_sim(2,i) xi_sim(2,i-1)]);%     
    pause(.00001)
%   else
%       return
 % end
  
end

