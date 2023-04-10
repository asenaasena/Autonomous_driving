function [theta] = wrap_theta(theta)
% wraps theta between -pi to pi
theta = mod(theta, 2*pi);
if theta >= pi
    theta = theta -2 *pi;
end