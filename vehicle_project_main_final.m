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

%% Run the initialization

%run('vehicle_project_UNCONSTRAINED.m');
load('20190528_u_opt_converge');

u_0                     =   u_opt;
clear u_opt

run('initial_guess_setting');

%% Initial Setting visualization

figure
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
     outerBoundary(:,2),'black'),grid on
axis equal

% hold on
% 
% for i=1:(n_states-1)
%    plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'.r');
% end

%% Constrained algorithm setting

myoptions               =   con_optimalset;
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.tolfun        =   1e-17;
myoptions.tolx          =	1e-16;
myoptions.ls_beta       =	0.8;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2*3;
myoptions.nitermax      =	200;
myoptions.xsequence     =	'on';

% (fun,x0,A,b,C,d,p,q,con_options,filename)

u_0                     =   [20;
                             25;
                             3/15*ones(T_end/Ts,1)];
                         
global num_dyn den_dyn num_int

s = tf('s');

CC_int                  =   90/s;
CC_int_d                =   c2d(CC_int, Ts_sim, 'tustin');
[num_int, den_int]      =   tfdata(CC_int_d, 'v');

CC_dyn                  =   920/(s/10 + 1);
CC_dyn_d                =   c2d(CC_dyn, Ts_sim, 'tustin');
[num_dyn, den_dyn]      =   tfdata(CC_dyn_d, 'v');

[u_opt,~,~,exit,seq]    =   con_NLP_opt(@(u)( fun(u,xi0, T_end, Ts, waypoints, n_wp , innerBoundary, outerBoundary,6)),u_0,[],[],[],[],1,3,myoptions,"con_iterations" );


%% Optimal trajectory plot

[xi, t_vec, ~,torque]   =   trajectory_generation_cc(u_opt, xi0, T_end, Ts,1e-2);
    
figure(10)
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
     outerBoundary(:,2),'black'),grid on
axis equal
hold on
plot(xi(1,:),xi(2,:))

figure(11)
plot(t_vec(1:end-1),torque);grid;title('Torque [Nm/s]');

figure(12)
plot(t_vec,xi(3,:));grid;title('Speed [m/s]');

figure(1)

for j = 1:size(seq,2)
    
    [xi_s,~,~,~]        =    trajectory_generation_cc(seq(:,j), xi0, T_end, 0.1,1e-2);
    plot(xi_s(1,:),xi_s(2,:));grid;
    hold on
    
end

%%