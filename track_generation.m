clear all
close all
clc

% create and plot track
track = oval_track()
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
n_iterations = N-1;

% plot of finish line and initialization the starting position
line([innerBoundary(2,1,1) outerBoundary(N,1,1)],[innerBoundary(2,2,1) outerBoundary(N,2,1)],'color','b','linewidth', 3) 
x   = [innerBoundary(1,1,1) outerBoundary(N,1,1)]';
y   = [innerBoundary(1,2,1) outerBoundary(N,2,1)]';
xy  = [mean(x) mean(y)]';
plot(xy(1),xy(2),'*r')


for i = 1:n_iterations
  target_position = [xy(1) xy(2)]';
  % random xy coordinates generation - must to be replaced with the real
  % update of the target position
  % x   = [innerBoundary(i+1,1,1) outerBoundary(N-i,1,1)]';
  % y   = [innerBoundary(i+1,2,1) outerBoundary(N-i,2,1)]';
  % xy  = [mean(x) mean(y)]';
  % this is the function we have to call each time there is an update of
  % the target position
  inside = track_constraints(target_position,N,innerBoundary,outerBoundary)
end

