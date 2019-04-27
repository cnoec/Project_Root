function [time, end_check] = sim_time(u_opt, innerBoundary, outerBoundary, inner_wl, outer_wl, xi_0, N)
%     Input: u_input, innerBoundary, outerBoundary, inner_wl, outer_wl, xi_0
%     Output: time, end_check: 0 vincolo violato, 1 arrivato alla fine


T_opt = u_opt(1:N)
delta_opt = u_opt((N+1):(2*N));

n_iterations        =       1000;   %worst case
boundary_number     =       1;
tau                 =       0;
d                   =       0;
Ts                  =       1e-2;

for i = 2:n_iterations %end of the iteration when we reach the final wl
    
  % current wayline selection  
  current_wl    =     current_wayline(inner_wl,outer_wl,boundary_number,innerBoundary,outerBoundary,n_wl,N)

%   if (current_wl == n_wl)
  % If we are in correspondence of the last wayline we have to keep the
  % optimal values until we reach the end of the track, otherwise we'll
  % have computational problem.
%       u_output          =           [T_opt(current_wl);delta_opt(current_wl)];
  
%   else
      
      % interpolator

      d_interpolated    =       interpolator_bws( inner_wl(current_wl,1:2),outer_wl(current_wl,1:2),inner_wl(current_wl+1,1:2),outer_wl(current_wl+1,1:2),xi_sim(1,i-1),xi_sim(2,i-1));

      T_kp1             =       T_opt(current_wl) + (T_opt(current_wl+1) - T_opt(current_wl))*d_interpolated;
      delta_kp1         =       delta_opt(current_wl) + (delta_opt(current_wl+1) - delta_opt(current_wl))*d_interpolated;

      u_output          =       [T_kp1;delta_kp1]
  
%   end
  
  % from the output of the interpolator to the new state
  xi_sim(:,i) = xi_sim(:,i-1) + Ts*Vehicle_Model_Function(0, xi_sim(:,i-1),u_output, 0, theta);
  
  % check if the target position is feasible
  [inside, boundary_number] = track_constraints(xi_sim(1,i),xi_sim(2,i),N,innerBoundary,outerBoundary,boundary_number)
 
  % if the target position is feasible, update of the state with the new
  % position. else exit the for cycle, because the trajectory is not
  % feasible.
  
  if inside == 1
     %plot([xi_sim(1,i) xi_sim(1,i-1)],[xi_sim(2,i) xi_sim(2,i-1)]);%     
  else
      time=-1;
      end_check = 0;
      return
  end
  
end

time = Ts*i;


