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

hold on

for i=1:(n_states-1)
   plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'.r');
end

%% Constrained algorithm setting

myoptions               =   con_optimalset;
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.8;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2*2;
myoptions.nitermax      =	5;
myoptions.xsequence     =	'on';
myoptions.QPoptions   =   ...
    optimset('Display','none','Algorithm','interior-point-convex','MaxIter',1000); 

% (fun,x0,A,b,C,d,p,q,con_options,filename)

% [u_opt,~,~,exit,seq]    = con_NLP_opt(@(u)( fun(u,xi0, T_end, Ts, waypoints, n_wp , innerBoundary, outerBoundary,6)),u_0,[],[],[],[],2,0,myoptions,"con_iterations" );


%% Optimal trajectory plot

[xi, ~, ~]              = trajectory_generation(u_opt, xi0, T_end, Ts);

figure
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on
axis equal

hold on
for i=1:(n_states-1)
   plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'.r');
end

%% Sequence plotting

for i = 1:min(size(seq))
    figure
    plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
        outerBoundary(:,2),'black'),grid on
    axis equal
    hold on
    u_check               =       seq(:,i);
    [xi_1, ~, ~]          =       trajectory_generation(u_check, xi0, T_end, Ts);
    plot(xi_1(1,:), xi_1(2,:),'.');grid;
    title(i);
    pause(1)
    close
end

%%