function newpos = Propagation(pos_UGV, variance)
global dt vel_cmd
% Position
x = pos_UGV(1);
y = pos_UGV(2);
psi = pos_UGV(3);

% Velocity command
speed      = vel_cmd(1) + sqrt(variance(1))*randn(1);
psi_dot    = vel_cmd(2) + sqrt(variance(2))*randn(1);

% Propagation equation
newpos(1,1) =  x+dt*cos(psi+dt*psi_dot)*speed;
newpos(2,1) =  y+dt*sin(psi+dt*psi_dot)*speed;
newpos(3,1) =  psi+dt*psi_dot;

end
