function [] = path_follow(trajectory,start_pt)
% This method calls run_circular_CCA and run_straight_CCA for each of the
% path planning step
  
for path_index = 1:length(trajectory)-1
    path.position = run_circular_CCA(trajectory(path_index),start_pt);
    start_pt = path.position(:,end)'; %  wrap theta angle
    start_pt(end) = wrap_theta(start_pt(end));
    path.position = run_straight_CCA(trajectory(path_index),start_pt);
    start_pt = path.position(:,end)';
    start_pt(end) = wrap_theta(start_pt(end)); %  wrap theta angle
end

end