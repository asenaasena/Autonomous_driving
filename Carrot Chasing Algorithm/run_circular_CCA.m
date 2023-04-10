function [trajectory] = run_circular_CCA(path_plan, initial_pt)

% lambda: Carrot Distance
global va umax Rmin dt kappa_circular lambda

t(1)                    =               0 ;                 % Simulation Time [s]

%.. Circular Orbit
O                       =               path_plan.center(1,:)' ;         % Centre of Orbit [m]
 
%.. Position and Velocity of Robot
x(1) = initial_pt(1,1) ;               % Initial Robot X Position [m]
y(1)  = initial_pt(1,2) ;                 % Initial Robot Y Position [m]
psi(1) =  initial_pt(1,3) ;           % Initial Robot Heading Angle [rad]
p(:,1)                  =               initial_pt(:,1:2)' ;   % Robot Position Initialization [m]

%% Path Following Algorithm
i                       =               0 ;                 % Time Index

while i<200 
    i                   =               i + 1 ;

    %==============================================================================%
    %.. Path Following Algorithm
    
    % Step 1
    % Distance between orbit and current Robot position, d
    d                   = norm(O - p(:,i)) - Rmin            ;
    
    % Step 2
    % Orientation of vector from initial waypoint to final waypoint, theta
    theta               = atan2(p(2,i)- O(2), p(1,i) - O(1)) ;
    
    % Step 3
    % Carrot position, s = ( xt, yt )
    xt                  = O(1)+Rmin * cos(theta+lambda) ;
    yt                  = O(2)+Rmin * sin(theta+lambda) ;
    
    % Step 4
    % Desired heading angle, psid
    psid                = atan2(yt-p(2,i), xt-p(1,i)) ;

    % Heading angle error, DEL_psi
    DEL_psi             = psid - psi(i);
    % Wrapping up DEL_psi
    DEL_psi             =               rem(DEL_psi,2*pi);
    if DEL_psi < -pi
        DEL_psi = DEL_psi + 2*pi;
    elseif DEL_psi > pi
        DEL_psi = DEL_psi-2*pi;
    end
    
    % Step 5
    % Guidance command, u
    u(i)                =   kappa_circular * DEL_psi * va      ;
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

    % Update the Robot Path
    trajectory(:,i+1)         =               [ x(i+1), y(i+1),psi(i+1) ]';

    %.. Time Update
    t(i+1)              =               t(i) + dt ;

    diff_vec = norm(p(:,i)'-path_plan.Tx(1:2),2);
    if diff_vec <= 0.8
        break
         
    end
end

%Trajectory Plot
figure(1) ;
hold on ;
grid on
plot( x, y, 'b' ,'LineWidth', 1.2 ) ;
hold on ;
xlabel('x (m)') ;
ylabel('y (m)') ;
legend( 'Actual Robot Trajectory','Desired Path', 'Location','northeast' ) ;
title('Robot Actual vs Desired Trajectory')
% 
end