function [sum_Delta] = MINIMIZATION_FUNCTION(u, xi0, T_end, Ts, waypoints, n_wp)

[xi, ~, ~] = trajectory_generation(u, xi0, T_end, Ts);

n_states = length(xi);
% 
% for i=1:(n_states-1)
%    plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'*r');
% end


dist = zeros(n_wp,1);
min_dist_point = zeros(n_wp,2);
min_index = 1;

for i=1:n_wp
    [dist(i),min_dist_point(i,:),j] = wp_to_trajectory_distance( waypoints(i,1:2)', xi(1:2,min_index:end), 'only' );
    min_index = min_index + j;
%     plot(min_dist_point(i,1),min_dist_point(i,2),'.b')
%     txt = {i};
%     text(min_dist_point(i,1)+1,min_dist_point(i,2)+1,txt);
%     disp(i);
%     display('Minimization function');
    disp([i j]);
end

sum_Delta = sum(dist);

end

