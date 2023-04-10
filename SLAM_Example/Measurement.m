function meas = Measurement(pos_UGV, pos_landmarks)
% Measurement model is represented in polar coordinates r and theta instead
% of x and y. atan2 is used to wrap the angle 

diffx = pos_landmarks(1,:) - pos_UGV(1,:);

diffy = pos_landmarks(2,:) - pos_UGV(2,:);
r = sqrt(diffx.^2 + diffy.^2);
angle = atan2(diffy, diffx);
%meas = pos_landmarks - pos_UGV(1:2);
meas = [r; angle];

end
