
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Path Following%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% This code performs Carrot Chasing Algorithm for the precalculated
%%% values in the assignment 2.1. For each waypoint, both exit and entry
%%% circle centers, entry angles, exit and entry locations are computed
%%% previously. This code generates and plots the robot's desired and the
%%% actual path with angles and the circles available. 

close all ;
clc ;
%% Initialization
global Rmin umax va start_point d2r dt kappa_circular kappa_straight delta lambda ;
%% General Parameters
%.. Angle Converting Parameters
r2d                     =               180 / pi ;          % Radian to Degree [-]
d2r                     =               1 / r2d ;           % Degree to Radian [-]
dt                      =               0.1 ;               % Time Step Size [s]

%% Robot Parameters
%.. Maximum Lateral Acceleration of Robot
Rmin                    =               5 ;                % Robot Minimum Turn Radius [m]
umax                    =               va^2 / Rmin ;       % Robot Maximum Lateral Acceleration [m]
va                      =               5 ;                % Robot Velocity [m/s]
start_point = [0 0 0];
%% Design Parameters For Circular
kappa_circular           =    50; %       50 %    5 ;              % Gain
lambda                   =     8; %      8 ;
%% Design Parameters For Straight
kappa_straight  = 4 %1.6 ;  
delta =  3 %3.2 ; 


%% Values that is calculated in the previous step 2.1
%% 
% For each waypoint, there are two centers for entry and exit circles 
centers = [[0, 5 ; 56, 63    ]; [56, 63 ; 82.5 ,115.7];[82.5 ,115.7 ; 145 ,70] ; [145 ,70 ; 95 ,32];[95 ,32 ; 50 ,5]];
entry_angles = [43 , 76.47, -32, 218, 208];
Txs = [3.84, 13.2 ; 61.58, 64.74; 84.9, 117; 146, 66.87; 95, 27 ];
Tns = [56.97, 63 ; 75.25, 121.58; 147, 75; 98, 28; 52.5,4];

% Assign the information above into struct
for i=1: size(entry_angles,2)
    trajectory(i).center = [centers(2*i -1,:);centers(2*i,:)] ;
    trajectory(i).position(1) = entry_angles(i);
    trajectory(i).Tx = Txs(i,:);
    trajectory(i).Tn = Tns(i,:);
end
%% Add the end point for plotting purposes
trajectory(i+1).center = [50,5;50,5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the entry exit circles and the exit angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% These waypoint position and heading angle values are given 

initial_pt = [0 10 0];
waypoint1 = [60 60 45];
waypoint2 = [80 120 30];
waypoint3 = [150 70 -90];
waypoint4 = [100 30 -120];
target = [50 0 -180];
waypoints = [initial_pt; waypoint1; waypoint2; waypoint3; waypoint4; target];

plot_desired_path(trajectory,waypoints)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PATH FOLLOWING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

path_follow(trajectory,start_point)



