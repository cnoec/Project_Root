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
current_wl = 0;
innerPosition = [innerBoundary(boundary_number,1,1) innerBoundary(boundary_number,2,1)];
outerPosition = [outerBoundary(boundary_number,1,1) outerBoundary(boundary_number,2,1)];

% since the outer boundary doesn't always correspond to the one that has
% the greatest coordinates, auxiliary variables are used: for the boundary 
% we are considering, x_out and x_in, y_out and y_in are identified. 
if innerPosition(1)>outerPosition(1)
   x_out = innerPosition(1);
   x_in  = outerPosition(1);
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

   
% for-cycle
for j = 1:n_wl
    for i = 1:n_wl

       % selection of inner and outer waylines i and i+1 coordinates
       inner_wl_i = [inner_wl(i,1,1) inner_wl(i,2,1)]';
       outer_wl_i = [outer_wl(i,1,1) outer_wl(i,2,1)]';
       if(i == n_wl) 
           i = 0; 
       end
       inner_wl_ip1 = [inner_wl(i+1,1,1) inner_wl(i+1,2,1)]';
       outer_wl_ip1 = [outer_wl(i+1,1,1) outer_wl(i+1,2,1)]';

       % since the outer point of the wayline doesn't always correspond to the 
       % one that has the greatest coordinates, auxiliary variables are used: 
       % for the two waylines we are considering, x_out and x_in, y_out and 
       % y_in are identified, both for waypoint i and waypoint i+1
       if inner_wl_i(1)>outer_wl_i(1)
           x_out_i = inner_wl_i(1);
           x_in_i =  outer_wl_i(1);
       else
           x_out_i = outer_wl_i(1);
           x_in_i  = inner_wl_i(1);
       end

       if inner_wl_i(2)>outer_wl_i(2)
           y_out_i = inner_wl_i(2);
           y_in_i  = outer_wl_i(2);
       else
           y_out_i = outer_wl_i(2);
           y_in_i  = inner_wl_i(2);
       end

       if inner_wl_ip1(1)>outer_wl_ip1(1)
           x_out_ip1 = inner_wl_ip1(1);
           x_in_ip1 =  outer_wl_ip1(1);
       else
           x_out_ip1 = outer_wl_ip1(1);
           x_in_ip1  = inner_wl_ip1(1);
       end

       if inner_wl_ip1(2)>outer_wl_ip1(2)
           y_out_ip1 = inner_wl_ip1(2);
           y_in_ip1  = outer_wl_ip1(2);
       else
           y_out_ip1 = outer_wl_ip1(2);
           y_in_ip1  = inner_wl_ip1(2);
       end

       % since the wayline i+1 doesn't always correspond to the 
       % one that has the greatest coordinates, auxiliary variables are used,
       % so that for the two waylines we are considering, the wayline i is the 
       % one having the smaller coordinates and the wayline i+1 the one with
       % the greates coordinates.
       if x_in_i>x_in_ip1
           aux = x_in_i;
           x_in_i = x_in_ip1;
           x_in_ip1 = aux;
       end
       if x_out_i>x_out_ip1
           aux = x_out_i;
           x_out_i = x_out_ip1;
           x_out_ip1 = aux;
       end

       if y_in_i>y_in_ip1
           aux = y_in_i;
           y_in_i = y_in_ip1;
           y_in_ip1 = aux;
       end
       if y_out_i>y_out_ip1
           aux = y_out_i;
           y_out_i = y_out_ip1;
           y_out_ip1 = aux;
       end

       % to see which wayline i is the last one the car has passed, we check
       % that the track boundaries corresponding to the car position are in
       % between the waylines' i and i+1.   
       if(x_in >= x_in_i && x_in <= x_in_ip1) || (x_out >= x_out_i && x_out <= x_out_ip1)
        if (y_in >= y_in_i && y_in <= y_in_ip1) || (y_out >= y_out_i && y_out <= y_out_ip1) 
           if (i == 0)
              i = n_wl; 
           end
           current_wl = i;
           return
        end
       end
    end
    
    if current_wl == 0
       inner_wl_i = [inner_wl(j,1,1) inner_wl(j,2,1)]';
       outer_wl_i = [outer_wl(j,1,1) outer_wl(j,2,1)]';
       if(j == n_wl) 
          j = 0; 
       end
       inner_wl_ip1 = [inner_wl(j+1,1,1) inner_wl(j+1,2,1)]';
       outer_wl_ip1 = [outer_wl(j+1,1,1) outer_wl(j+1,2,1)]';
        
        if inner_wl_i(1)>outer_wl_i(1)
           x_out_i = inner_wl_i(1);
           x_in_i =  outer_wl_i(1);
       else
           x_out_i = outer_wl_i(1);
           x_in_i  = inner_wl_i(1);
       end

       if inner_wl_i(2)>outer_wl_i(2)
           y_out_i = inner_wl_i(2);
           y_in_i  = outer_wl_i(2);
       else
           y_out_i = outer_wl_i(2);
           y_in_i  = inner_wl_i(2);
       end

       if inner_wl_ip1(1)>outer_wl_ip1(1)
           x_out_ip1 = inner_wl_ip1(1);
           x_in_ip1 =  outer_wl_ip1(1);
       else
           x_out_ip1 = outer_wl_ip1(1);
           x_in_ip1  = inner_wl_ip1(1);
       end

       if inner_wl_ip1(2)>outer_wl_ip1(2)
           y_out_ip1 = inner_wl_ip1(2);
           y_in_ip1  = outer_wl_ip1(2);
       else
           y_out_ip1 = outer_wl_ip1(2);
           y_in_ip1  = inner_wl_ip1(2);
       end
       
       if (y_in > y_in_ip1 && y_in > y_in_i) && (x_in >= x_in_i && x_in <= x_in_ip1)
           if (j == 0)
              j = n_wl; 
           end
           current_wl = j;
           return
       elseif (y_in < y_in_ip1 && y_in < y_in_i) && (x_in >= x_in_i && x_in <= x_in_ip1)
           if (j == 0)
              j = n_wl; 
           end
           current_wl = j;
           return
       elseif (x_in > x_in_ip1 && x_in > x_in_i)&&(y_in >= y_in_i && y_in <= y_in_ip1)
           if (j == 0)
              j = n_wl; 
           end
           current_wl = j;
           return
      elseif (x_in < x_in_ip1 && x_in < x_in_i)&&(y_in >= y_in_i && y_in <= y_in_ip1)
           if (j == 0)
              j = n_wl; 
           end
           current_wl = j;
           return
    end
end
end

