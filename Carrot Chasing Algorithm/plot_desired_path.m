function [] = plot_desired_path(trajectory,waypoints)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global Rmin start_point d2r
t = linspace(0, 2*pi, 100);
figure;
hold on;

for i= 1:length(trajectory)-1

% Compute the endpoint of the line segment
x = waypoints(i,1);
y = waypoints(i,2);
theta = waypoints(i,3);
x2 = x + Rmin * cosd(theta);
y2 = y + Rmin * sind(theta);
plot(x, y, 'b.', 'MarkerSize', 10);  % plot the point as a red circle with size 10
plot([x x2], [y y2]);

% plotting the first (exit) circles 
plot(trajectory(i).center(1,1)+Rmin*cos(t), trajectory(i).center(1,2)+Rmin*sin(t), 'r--'); 

% plotting the second (entry) circles
plot(trajectory(i).center(2,1)+Rmin*cos(t), trajectory(i).center(2,2)+Rmin*sin(t), 'b--');

end


origin = [start_point(1) start_point(2) start_point(3)*d2r];
plot(origin(1), origin(2), 'm.', 'MarkerSize', 10)
text(origin(1) -1, origin(2)-2, 'origin')
p2 = origin(1) + Rmin*cosd(origin(3));
q2 = origin(2) + Rmin*sind(origin(3));
plot([origin(1) p2], [origin(2) q2]);
grid on

end