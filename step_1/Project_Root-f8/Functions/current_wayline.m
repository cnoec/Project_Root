function current_wl = current_wayline(inner_wl,outer_wl,boundary_number,innerBoundary,outerBoundary,n_wl,N)
  
% function that returns the number of the wayline the car has just passed.
%
% Inputs:   inner_wl            -   matrix n_wlx3 - it contains the x,y,z
%                                   coordinates of the inner wayline
%                                   samples.
%           outer_wl            -   matrix n_wlx3 - it contains the x,y,z
%                                   coordinates of the outer wayline
%                                   samples.
%           boundary_number     -   position in the inner boundary matrix
%                                   of the coordinates corresponding to the
%                                   last car position                                   
%           innerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the outer boundary
%                                   samples.
%           n_wl                -   number of wayline samples 
%           N                   -   number of boundary samples
%
% Outputs:  current_wl          -   it returns the number of last wayline
%                                   the car has passed
%

% initialization of variables and boundary coordinates selection
current_wl = 1;
innerPosition = [innerBoundary(boundary_number,1,1) innerBoundary(boundary_number,2,1)];
outerPosition = [outerBoundary(boundary_number,1,1) outerBoundary(boundary_number,2,1)];

x_in = innerPosition(1);
y_in = innerPosition(2);
%plot(x_in,y_in,'*r')
for i = 1:n_wl
   
   % selection of inner and outer boundaries' coordinates of the waylines 
   % i and i+1
   innerPosition_i   = [inner_wl(i,1,1) inner_wl(i,2,1)]';
   outerPosition_i   = [outer_wl(i,1,1) outer_wl(i,2,1)]';
   if(i == n_wl) 
           i = 0; 
       end
   innerPosition_ip1 = [inner_wl(i+1,1,1) inner_wl(i+1,2,1)]';
   outerPosition_ip1 = [outer_wl(i+1,1,1) outer_wl(i+1,2,1)]';
   
   %creation of the polygon in which the car has to be.
   polygon_1 = [innerPosition_i(1) outerPosition_i(1) innerPosition_ip1(1) outerPosition_ip1(1)];
   polygon_2 = [innerPosition_i(2) outerPosition_i(2) innerPosition_ip1(2) outerPosition_ip1(2)];
   k = convhull(polygon_1,polygon_2); 
   % plot(polygon_1(k),polygon_2(k),'r-','linewidth',1)
      
   % to be inside the track, the inner boudary corresponding to the current 
   % position must be inside the polygon formed by the inner and out 
   % boundaries of the previous and following wayline  
   [inside_pol,on_pol] = inpolygon(x_in,y_in,polygon_1(k),polygon_2(k));
   
    % output variable setting
    if (inside_pol >= 1)&&(on_pol == 0)
        if(i==0)
            i = n_wl;
        end
        current_wl = i;
        return;
    elseif (on_pol >= 1)
         if(i==0)
            i = 1;
        end
        current_wl = i+1;
        return;
    end
    
end


end

