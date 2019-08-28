function [xi, t_vec, end_check] = trajectory_generation_sim(u, xi_0, T_end, Ts)
%       Input:  u: input (torque, delta)
%               xi_0: initial conditions
%               T_end: simulation time
%               Ts: time step for the numerical simulation
%                     
%       Output: xi: state of the system wrt time
%               t_vec: time vector
%               end_check:  1: ok
%                           0: input di dimensione errata
%       

%% vettore del tempo

t_vec                                   =   0:Ts:T_end;
t_vec                                   =   t_vec';
N                                       =   length(t_vec); 
Ts_1ms                                  =   1e-3;

%% dimensional check

end_check   = 1;

if N+4 ~= length(u)
    end_check =  0;
    disp('Errore dimensione');
    xi=0;
    return
end

%% Input setting

load('parameters.mat');

steer_input                             =   timeseries(u(5:end),t_vec);
speed_ref                               =   zeros(N,1);
speed_ref(1:floor(N/4))                 =   u(1);
speed_ref(floor(N/4)+1:floor(N/2))      =   u(2);
speed_ref(floor(N/2)+1:floor(3*N/4))    =   u(3);
speed_ref(floor(3*N/4)+1:end)           =   u(4);

speed_ref                               =   timeseries(speed_ref,t_vec);

%% cruise control

s = tf('s');

CC_int                                  =   90/s;
CC_int_d                                =   c2d(CC_int, Ts_1ms, 'tustin');
[num_int, den_int]                      =   tfdata(CC_int_d, 'v');

CC_dyn                                  =   920/(s/10 + 1);
CC_dyn_d                                =   c2d(CC_dyn, Ts_1ms, 'tustin');
[num_dyn, den_dyn]                      =   tfdata(CC_dyn_d, 'v');

%% simulation

% When using variable mask parameters in Simulink, the base workspace is 
% the default source workspace of Simulink

simout  = sim('trajectory_simulation.slx', 'SrcWorkspace', 'current');

xi      = simout.xi.signals.values';
t_vec   = simout.tout;

%%









