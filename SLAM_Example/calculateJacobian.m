function mat = calculateJacobian(pos_UGV, pos_landmarks)
%Jacobian of range and bearing measurements wrt to x and y

diffx = pos_landmarks(1,:) - pos_UGV(1,:);
diffy = pos_landmarks(2,:) - pos_UGV(2,:);
sum_ = diffx.^2 + diffy.^2 ; 
sum_sqr = sqrt(sum_);
mat = [diffx./sum_sqr diffy./sum_sqr ; 
    - diffy./sum_ diffx./sum_];


end