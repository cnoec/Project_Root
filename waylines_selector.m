function [inner_wl, outer_wl, n_wl] = waylines_selector(innerBoundary,outerBoundary, max_distance)
% This function computes the different wayline
%
%     INPUT:  innerBoundary: vector of the inner coordinates
%             outerBoundary: vector of the outer coordinates
%             max_distance : maximum distance between two waylines
%     OUTPUT: inner_wl: vector of the inner coordinates of the waylines
%             outer_wl: vector of the outer coordinates of the waylines
%             n_wl:     number of waylines
%             

% random point to identify the circumefernces

A = [innerBoundary(50,1) innerBoundary(50,2) outerBoundary(70,1) outerBoundary(70,2);
    innerBoundary(100,1) innerBoundary(100,2) outerBoundary(140,1) outerBoundary(140,2);
    innerBoundary(150,1) innerBoundary(150,2) outerBoundary(210,1) outerBoundary(210,2);];

[R,xcyc] = fit_circle_through_3_points(A);
% x_circle = (xcyc(1,1)+xcyc(1,2))/2;
% y_circle = (xcyc(2,1)+xcyc(2,2))/2;

n_wl = ceil(2*pi*max(R(1),R(2))/max_distance);
n_vec = length(innerBoundary(:,1));

inner_wl = zeros(n_wl,3);
outer_wl = zeros(n_wl,3);

for ind=0:(n_wl-1)
    inner_wl(ind+1,:) = innerBoundary(ind*floor(n_vec/n_wl)+1,:);
    outer_wl(ind+1,:) = outerBoundary(ind*floor(n_vec/n_wl)+1,:);
end
