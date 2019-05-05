function dist = wp_to_trajectory_distance( target, traject, cmd )
% wp_to_trajectory_distance computes the minimum distance between the
% target points and the trajectory. It is also possible to set a string cmd
% in order to compute the minimum distance from all the target points.
% INPUTS
%       target:      vector conteining the target points
%       trajectory:  matrix conteining the trajectory, on the first column
%                    we'll find the x coordinate, while on the second one 
%                    we'll have the y ones.
%       cmd:         string which allows the user to use this function to
%                    compute:
%                               -'all' : the function will return a vector
%                                        composed by the minimum distances
%                                        from all the target points.
%                               -'only': the function will return a scalar
%                                        which represents the minimum
%                                        distance between the input target
%                                        point and the input trajectory.
%
% OUTPUTS
%       ditance:    minimum distance

T       =       length( traject(:,1) );
N       =       length( target );

if ( strmcp( cmd, 'only' ) )

    % INITIALIZATION
    dist        =       norm( target - traject(1,:) );

    % MINIMUM DISTANCE COMPUTATION
    for j = 2 : T
        
        if ( norm( target - traject(j,:) ) < dist )
            
            dist    =       norm( target - traject(j,:) );
            
        end
        
    end

    
elseif ( strmcp( cmd, 'all' ) )
    
    dist        =       zeros( N,1 );
    
    % INITIALIZATION
    for j = 0:N
        
        dist(j)     =       norm( target(j) - traject( 1,: ) );
        
    end
    
    % MINIMUM DISTANCE COMPUTATION
    % first for cycle in order to run over all the target points
    for j = 0 : N

        % the second for cycle is needes in order to select for each target
        % point the minimum distance from the trajectory
        for i = 0 : T
            
            if ( norm( target(j) - traject(i,:) ) < dist(j) )
            
                dist(j)    =       norm( target(j) - traject(i,:) );
            
            end
            
        end
        
    end
    
end

end