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

%% Run the initialization

%run('Vehicle_Project_main_torque.m');
load('20190528_u_opt_converge');

u_0         =   u_opt;
clear u_opt

%% Initial guess

run('initial_guess_setting');

%% Initial Setting visualization

figure
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal

hold on

for i=1:(n_states-1)
   plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'.r');
end

%% Constrained algorithm setting

myoptions               =   con_optimalset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';

% (fun,x0,A,b,C,d,p,q,con_options,filename)

[u_opt,~,~,exit,seq] = con_NLP_opt(@(u)( fun(u,xi0, T_end, Ts, waypoints, n_wp)),u_0,[],[],[],[],2,0,myoptions,"con_iterations" );

%%