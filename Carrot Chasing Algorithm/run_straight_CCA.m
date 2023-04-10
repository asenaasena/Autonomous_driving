function [trajectory] = run_straight_CCA(path_plan, initial_pt)
%% Parameters
global radius va umax kappa_straight delta Rmin
%.. Angle Converting Parameters

dt = 0.1 ;               % Time Step Size [s]
t(1) = 0 ;                 % Simulation Time [s]
pos(:,1) = initial_pt;

pt_1 = path_plan.Tx;
pt_2= path_plan.Tn;

%.. Waypoint
Wi = pt_1(:,1:2)' ;         % Initial Waypoint Position [m]
Wf  = pt_2(:,1:2)' ;     % Final Waypoint Position [m]

% %.. Position and Velocity of Robot
x(1) = initial_pt(1,1) ;               % Initial Robot X Position [m]
y(1)  = initial_pt(1,2) ;                 % Initial Robot Y Position [m]
psi(1) =  initial_pt(1,3) ;           % Initial Robot Heading Angle [rad]
p(:,1) = initial_pt(:,1:2)' ;   % Robot Position Initialization [m]

%% Path Following Algorithm
i                       =               0 ;                 % Time Index
while (i+1)<500                 % norm(Wf - p(:,1))<= 10
    i = i + 1 ;
    %==============================================================================%
    %.. Path Following Algorithm

    % Distance between initial waypoint and current Robot position, Ru
    Ru = norm(Wi- p(:,i)); % sqrt(((wi(1)-p(1))^2+ ((wi(2)-p(2))^2)             
    % Orientation of vector from initial waypoint to final waypoint, theta
    theta = atan2((Wf(2)-Wi(2)),(Wf(1)-(Wi(1))))          ;
    
    % Orientation of vector from initial waypoint to current Robot position, thetau
    thetau              = atan2((p(2,i)-Wi(2)), p(1,i)-Wi(1))            ;
    % Difference between theta and theatu, DEL_theta
    DEL_theta           = theta - thetau ;

    % Distance between initial waypoint and q, R
    R                   = sqrt(Ru^2 - (Ru*sin(DEL_theta))^2) ;
    
    % Carrot position, s = ( xt, yt )
    xt                  = Wi(1) + (R+delta)*cos(theta) ;
    yt                  = Wi(2) + (R+delta)*sin(theta) ;
    
    % Desired heading angle, psid. atan2 returns values between -pi to pi
    psid                = atan2(yt-p(2,i), xt-p(1,i));

    % Guidance command, u
    u(i)                = kappa_straight * (psid-psi(i))*va         ;
    % Limit u
    if u(i) > umax
        u(i)            =               umax;
    elseif u(i) < -umax
        u(i)            =             - umax;
    end
    %==============================================================================%
    
    %.. Robot Dynamics
    % Dynamic Model of Robot
    dx                  =               va * cos( psi(i) ) ;
    dy                  =               va * sin( psi(i) ) ;
    dpsi                =               u(i) / va ;
    
    % Robot State Update
    x(i+1)              =               x(i) + dx * dt ;
    y(i+1)              =               y(i) + dy * dt ;
    psi(i+1)            =               psi(i) + dpsi * dt ;
    
    % Robot Position Vector Update
    p(:,i+1)            =               [ x(i+1), y(i+1) ]' ;
    trajectory(:,i+1)         =               [x(i+1), y(i+1),psi(i+1) ]';

    %.. Time Update
    t(i+1)              =               t(i) + dt ;


    diff = norm(Wf - p(1:2,i));
    if diff<=0.5
        break
    end
end


%% Result Plot
%.. Trajectory Plot
figure(1) ;
plot( [ Wi(1), Wf(1) ], [ Wi(2), Wf(2) ], 'r', 'LineWidth', 0.8 ) ;
hold on ;
plot( x, y, 'b', 'LineWidth',  1.5) ;
hold on ;
xlabel('X (m)') ;
ylabel('Y (m)') ;
legend('Actual Robot Trajectory','Desired Path','Location', 'northwest' ) ;
