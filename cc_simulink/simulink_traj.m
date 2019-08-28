function [xi, t_vec, end_check] = simulink_traj(u, xi_0, T_end, Ts)
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
%       Questa funzione simula a passo costante (eulero in avanti) dati
%       ingressi e tempo di simulazione, fino a T_end-Ts.
%
%       NOTA BENE: L'INPUT E' INTESO COME PRIMA TUTTE LE COPPIE E DOPO
%       TUTTI GLI ANGOLI, DITEMI SE E' DA CAMBIARE

%% vettore del tempo

t_vec = 0:Ts:T_end;
t_vec = t_vec';
N = length(t_vec); 
Ts_1ms = 1e-3;

%% dimensional check

end_check = 1;

if N+1 ~= length(u)
    end_check =  0;
    disp('Errore dimensione');
    xi=0;
    return
end

%%

run('Parameters.m');

steer_input = timeseries(u(2:end),t_vec);
speed_ref = u(1);

%% cruise control

s = tf('s');

CC_int = 90/s;
CC_int_d = c2d(CC_int, Ts_1ms, 'tustin');
[num_int, den_int] = tfdata(CC_int_d, 'v');

CC_dyn = 920/(s/10 + 1);
CC_dyn_d = c2d(CC_dyn, Ts_1ms, 'tustin');
[num_dyn, den_dyn] = tfdata(CC_dyn_d, 'v');

%% sim

% When using variable mask parameters in Simulink, the base workspace is the default source workspace of Simulink

simout = sim('simulaz.slx', 'SrcWorkspace', 'current');

xi = simout.xi.signals.values;
t_vec = simout.tout;









