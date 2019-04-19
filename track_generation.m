function [track,innerBoundary,outerBoundary,N] = track_generation()

% auxiliary function that generates the track and sets the initial
% position. change the function called by track to change the shape.
%
% Outputs:  track               -   matrix  
%           innerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the outer boundary
%                                   samples. 
%           x0                  -   initial x coordinate
%           y0                  -   initial y coordinate
%           N                   -   number of boundary samples


% create and plot track
track = circle_track()
plot(track),hold on
camroll(-90)

% find inner and outer boundaries coordinates
rb = roadBoundaries(track)
outerBoundary = rb{2};
innerBoundary = rb{1};

% figure
% plot(innerBoundary(:,1),innerBoundary(:,2),'b', outerBoundary(:,1),outerBoundary(:,2),'b'),grid on
% axis equal
        
N = length(innerBoundary);
