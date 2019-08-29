function [xi, t_vec, end_check,Torque] = trajectory_generation_cc(u, xi_0, T_end, Ts_steer, Ts_sim)
%       Input:  u: input (torque, delta)
%               xi_0: initial conditions
%               T_end: simulation time
%               Ts_steer: sampling time for the steering
%               Ts_sim: sampling time for the simulation
%                     
%       Output: xi: state of the system wrt time
%               t_vec: time vector
%               end_check:  1: ok
%                           0: input di dimensione errata
%                Torque: vettore delle coppie
%       
%       Questa funzione simula a passo costante (eulero in avanti) dati
%       ingressi e tempo di simulazione, fino a T_end-Ts.
%
%       NOTA BENE: L'INPUT E' INTESO COME PRIMA TUTTE LE COPPIE E DOPO
%       TUTTI GLI ANGOLI, DITEMI SE E' DA CAMBIARE

%% vettore del tempo

% path        =       pwd;
% addpath( '..\Model' );
% addpath( '..' );

end_check = 1;
t_vec = 0:Ts_sim:(T_end-Ts_sim);
t_vec = t_vec';
N = length(t_vec);
downsampling = Ts_steer/Ts_sim;
n_ref = 2;

for ind=1:(T_end/Ts_steer)
    steer(((ind-1)*downsampling+1):ind*downsampling) = u(n_ref+ind);
end

steer = steer.';

%% dimensional check

if  length(steer) ~= length(t_vec)
    end_check =  0;
    xi=0;
    disp('Errore dim');
    return
end

xi = zeros(6,N);
xi(:,1) = xi_0;

load('parameters.mat');

%% simulazione

eps = 0;

for ind=2:N
    if  ind < floor(N/2)
         if xi(3,ind-1) <= u(1) - eps
             T_in = Tdmax;
         elseif xi(3,ind-1) >= u(1) + eps
             T_in = -Tdmax;
         elseif (xi(3,ind-1) > u(1) - eps) && (xi(3,ind-1) < u(1) + eps)
             T_in = 0;
         end
    else
         if xi(3,ind-1) <= u(2) - eps
             T_in = Tdmax;
         
         elseif xi(3,ind-1) >= u(2) + eps
             T_in = -Tdmax;
         elseif (xi(3,ind-1) > u(2) - eps) && (xi(3,ind-1) < u(2) + eps)
             T_in = 0;
         end 
    end
    Torque(ind-1) = T_in;
    input = [T_in; steer(ind);];
    xi(:,ind) = xi(:,ind-1)  + Ts_sim*Vehicle_Model_Function(xi(:,ind-1), input, theta);
end

