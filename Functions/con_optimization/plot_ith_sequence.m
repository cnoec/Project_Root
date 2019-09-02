function plot_ith_sequence(seq,i,innerBoundary,outerBoundary,xi0,T_end)

figure(99)
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
     outerBoundary(:,2),'black'),grid on
axis equal
hold on

[xi_s,~,~,~]        =    trajectory_generation_cc(seq(:,i), xi0, T_end, 0.1,1e-2);

plot(xi_s(1,:),xi_s(2,:))


end

