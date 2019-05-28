function [sum_Delta] = deltasum_Edo(u, xi0, T_end, Ts, waypoints, n_wp,gamma,plot_option)
n_iterations = T_end/Ts;

u(1:n_iterations)*890;

[xi, ~, ~] = trajectory_generation(u, xi0, T_end, Ts);



dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_iterations);

sum_Delta = sum(dist);
penalty = zeros(n_iterations-1,1);

for i = 1:n_iterations-1
    penalty(i) = sqrt((xi(1,i)-xi(1,i+1))^2 + (xi(2,i)-xi(2,i+1))^2);
end

if plot_option
    sum_Delta
    gamma*sum(penalty)
end

sum_Delta = sum_Delta + gamma*sum(penalty);

end