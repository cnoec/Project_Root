function [inside_track,boundary_number] = track_constraints(x_target,y_target,N,innerBoundary,outerBoundary,boundary_number)

% constraint function h(x)>=0. It checks whether or not the target position
% is inside the track and if it is feasible, by returning 1 or -1. 
%
% Inputs:   x_target            -   coordiate x of target position
%           y_target            -   coordinate y of target position
%           N                   -   number of boundary samples
%           innerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the outer boundary
%                                   samples. 
%           boundary_number     -   position in the inner boundary matrix
%                                   of the coordinates corresponding to the
%                                   last car position                                   
%
% Outputs:  inside_track        -   if  1 -> target position inside track
%                                   if -1 -> target position outside track
%           boundary_number     -   position in the inner boundary matrix
%                                   of the coordinates corresponding to the
%                                   last car position                                   

% initialization of variables 
inside_track = 0;

% for-cycle
% the cycle starts from boundary_number = 1, that is where the finish line
% is. then it is updated so that the car can go only in the forward
% direction
for i = 1:N
   
   % selection of inner and outer boundaries' coordinates at track sample 
   % i and i+1
   if(i == N) 
       innerPosition_i   = [innerBoundary(i,1,1) innerBoundary(i,2,1)]';
       outerPosition_i   = [outerBoundary(i,1,1) outerBoundary(i,2,1)]';
       i = 1; 
       innerPosition_ip1 = [innerBoundary(i+1,1,1) innerBoundary(i+1,2,1)]';
       outerPosition_ip1 = [outerBoundary(i+1,1,1) outerBoundary(i+1,2,1)]';
   else
       innerPosition_i   = [innerBoundary(i,1,1) innerBoundary(i,2,1)]';
       outerPosition_i   = [outerBoundary(i,1,1) outerBoundary(i,2,1)]';
       innerPosition_ip1 = [innerBoundary(i+1,1,1) innerBoundary(i+1,2,1)]';
       outerPosition_ip1 = [outerBoundary(i+1,1,1) outerBoundary(i+1,2,1)]';
   end
   %creation of the polygon in which the car has to be.
   
   polygon_1 = [innerPosition_i(1) outerPosition_i(1) innerPosition_ip1(1) outerPosition_ip1(1)];
   polygon_2 = [innerPosition_i(2) outerPosition_i(2) innerPosition_ip1(2) outerPosition_ip1(2)];
   k = convhull(polygon_1,polygon_2); 
   % plot(polygon_1(k),polygon_2(k),'g-','linewidth',1)
      
   % to be inside the track, the target position must be inside the polygon
   % formed by the inner and out boundaries and the track.   
   inside_track = inpolygon(x_target,y_target,polygon_1(k),polygon_2(k));
   
    % output variable setting
    if inside_track >= 1
        inside_track = 1;
        boundary_number = i;
        return;
    else 
        inside_track = -1;
    end
end
