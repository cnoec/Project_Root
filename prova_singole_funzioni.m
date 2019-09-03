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

T_end = 25;
Ts_sim = 1e-2;
Ts_steer = 1e-1;

%% trova funzioni
u_opt = [20; 25; 3/15*ones(T_end/Ts_steer,1);];
[xi, ~, ~,~] = trajectory_generation_cc(u_opt, xi0, T_end, Ts_steer, Ts_sim);

%%
Par_3 = CircleFitByTaubin(outerBoundary(30:98,:));
Par_4 = CircleFitByTaubin(innerBoundary(30:98,:));

Par_7 = CircleFitByTaubin(outerBoundary(158:228,:));
Par_8 = CircleFitByTaubin(innerBoundary(158:228,:));

Par_1 = -Par_3(3)+Par_4(2);
Par_2 = -Par_4(3)+Par_4(2);

Par_5 = Par_3(3)+Par_4(2);
Par_6 = Par_4(3)+Par_4(2);

n_states = T_end/Ts_sim;

tic
h_track = zeros(2*n_states/10,1);
eps = 0.05;

for ind = 10:10:n_states

    state = [xi(1,ind) xi(2,ind)];      %punto da verificare
    
if state(1) > Par_4(1)
    centre_x = mean([Par_3(1) Par_4(1)]);
    centre_y = mean([Par_3(2) Par_4(2)]);
    
    dist = sqrt( (centre_x-state(1))^2 + (centre_y-state(2))^2 );   % la distanza tra la traiettoria ed il centro della circ
    h_track(((ind/10-1)*2+1):ind*2/10) = [   -dist+Par_3(3)-eps;           % settori A,B
                                        dist-Par_4(3)-eps];
%      disp('A,B');
         
elseif state(1) > Par_8(1) && state(1) <= Par_4(1)
    dist = state(2);            % la distanza è la coordinata y
    if state(2) > Par_4(2)      % settore C
        h_track(((ind/10-1)*2+1):ind/10*2) = [-dist+Par_5(1)-eps; dist-Par_6(1)+eps;];
%         disp('C');
    else
        h_track(((ind/10-1)*2+1):ind/10*2) = [-dist+Par_2(1)-eps; dist-Par_1(1)+eps;]; 
%         disp('F');      % settore F
    end
      
elseif state(1) < Par_8(1)
    centre_x = mean([Par_7(1) Par_8(1)]);
    centre_y = mean([Par_7(2) Par_8(2)]);
    dist = sqrt( (centre_x-state(1))^2 + (centre_y-state(2))^2 ); 
    h_track(((ind/10-1)*2+1):ind/10*2) = [-dist+Par_7(3)-eps;           % settori D,E
              dist-Par_8(3)-eps];
%     disp('D,E');
    
end


end
edo = toc

%%

tic 
track_constraint(xi,innerBoundary,outerBoundary,n_states,road_width);
sofi = toc


%%

figure
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),outerBoundary(:,2),'black'),grid on, axis equal;
hold on

for ind = 1:n_states
    if h_track((ind-1)*2+1) >= 0 && h_track(ind*2) >= 0
        plot(xi(1,ind), xi(2,ind),'.g');
        hold on
    else
        plot(xi(1,ind), xi(2,ind),'.r');
        hold on
    end
end

hold off

%%

h = track_constraint(xi,innerBoundary,outerBoundary,T_end/Ts_sim,6);


%% parameters initialization and setting of initial state

m = (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m = -1/m;

X       =       2;         % inertial X position (m)
Y       =       -50;         % inertial Y position (m)
Ux      =       20;         % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       atan(m);    % yaw angle (rad)
r       =       0;          % yaw rate (rad/s)

xi0     =       [X Y Ux beta psi r]';

%% time

Ts = 1e-1;
Ts_sim = 1e-2;
T_end = 25;
N = T_end/Ts +1;

%%

global num_dyn den_dyn num_int

tic
s = tf('s');

CC_int                                  =   90/s;
CC_int_d                                =   c2d(CC_int, Ts_sim, 'tustin');
[num_int, den_int]                      =   tfdata(CC_int_d, 'v');

CC_dyn                                  =   920/(s/10 + 1);
CC_dyn_d                                =   c2d(CC_dyn, Ts_sim, 'tustin');
[num_dyn, den_dyn]                      =   tfdata(CC_dyn_d, 'v');
toc

%%
u = [30; 30; 3/15*ones(N,1);];

[xi, t_vec, exitflag,Torque] = trajectory_generation_cc(u, xi0, T_end, Ts, Ts_sim);
figure(1)
plot(xi(1,:), xi(2,:)),grid on;

figure
plot(t_vec, xi(3,:)), grid on;

figure
plot(t_vec(1:(end-1)), Torque), grid on;


%% test cost function 1
tic
cost = deltasum(u, xi0, T_end, Ts, waypoints, n_wp);
cost_fun_time = toc

%% test cost function 2
tic
[cost_2, point_index] = unc_MINFUNC_gamma(u, xi0, T_end, Ts, waypoints, n_wp, 10)
toc

%% gradiente

tic
[y, grad] = gradient_num_comp((@(u)deltasum(u, xi0,T_end,Ts,waypoints, n_wp)), u, 'FD', 2^-17);
grad_time = toc
