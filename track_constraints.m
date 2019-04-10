
function inside_track = track_constraints(target_position,N,innerBoundary,outerBoundary)

% constraint function h(x)>=0. It checks whether or not the target position
% is inside the track and if it is feasible, by returning 1 or -1. 
%
% Inputs:   target_position     -   column vector 2x1 [ x y ]' 
%           N                   -   number of boundary samples
%           innerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the outer boundary
%                                   samples. 
%
% Outputs:  inside_track        -   if  1 -> target position inside track
%                                   if -1 -> target position outside track

% initialization of variables 
x = target_position(1);
y = target_position(2);
inside_track = 0;

% for-cycle
for i = 1:N
   j = N+1-i;
   
   % selection of inner and outer boundaries' coordinates at track sample i
   innerPosition = [innerBoundary(i,1,1) innerBoundary(i,2,1)]';
   outerPosition = [outerBoundary(j,1,1) outerBoundary(j,2,1)]';
   
   % since the outer boundary doesn't always correspond to the one that has
   % the greatest coordinates, auxiliary variables are used: for each 
   % sample i, x_out and x_in, y_out and y_in are identified. 
   if innerPosition(1)>outerPosition(1)
       x_out = innerPosition(1);
       x_in =  outerPosition(1);
   else
       x_out = outerPosition(1);
       x_in  = innerPosition(1);
   end
   
   if innerPosition(2)>outerPosition(2)
       y_out = innerPosition(2);
       y_in  = outerPosition(2);
   else
       y_out = outerPosition(2);
       y_in  = innerPosition(2);
   end

   % to be inside the track, the target position must have x coordinates in
   % between x_out and x_in and y coordinates in between y_out and y_in. 
   % at the end of the for-cycle, if the target position is feasible, the
   % inside_track variable will be greater than zero. else, the target
   % position is not feasible
   if (x >= x_in && x <= x_out)
     if (y >= y_in && y <= y_out)
       inside_track = inside_track + 1;
     end
   end
end

% output variable setting
if inside_track >= 1
    inside_track = 1;
    plot(x,y,'*r')
    pause(.001)
else 
    inside_track = -1;
end

end