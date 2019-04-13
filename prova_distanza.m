close all
clear all
clc


% v1 = [0,0,0];
% v2 = [3,0,0];
% pt = [0,5,0;0,10,0;0,15,0];
% distance_3D = point_to_line_distance(pt, v1, v2)

v1 = [3,0];
v2 = [3,3];
pt = [0,5;0,10;0,15];
distance_2D = point_to_line_distance(pt, v1, v2)

plot(v1(1,1),v1(1,2),'*',v2(1,1),v2(1,2),'r*',pt(1,1),pt(1,2),'g*',pt(2,1),pt(2,2),'g*',pt(3,1),pt(3,2),'g*');grid;axis([ 0 20 -20 20 ]);