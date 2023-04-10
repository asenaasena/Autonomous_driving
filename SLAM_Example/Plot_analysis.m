%% UGV Trajectory
figure; hold on;
title('UGV Trajectories');
plot(real_UGV1(1,:),real_UGV1(2,:),'r');
plot(real_UGV2(1,:),real_UGV2(2,:),'b');
plot(pos_UGV1(1,:),pos_UGV1(2,:),'r:');
plot(pos_UGV2(1,:),pos_UGV2(2,:),'b:');
grid on; xlabel('x (m)'); ylabel('y (m)');
axis([-30, 30, -30, 30]);
legend('True UGV1','True UGV2','Estimated UGV1','Estimated UGV2');

%% UGV Position Error
error_UGV1 = pos_UGV1-real_UGV1;
error_UGV2 = pos_UGV2-real_UGV2;
rmse_UGV1 = sqrt(sum(error_UGV1.^2,1));
rmse_UGV2 = sqrt(sum(error_UGV2.^2,1));

figure; 
set(gcf,'position',[0 0 500 1000]);
subplot(411); hold on;
title('UGV Estimation Error');
plot(time,rmse_UGV1,'r');
plot(time,rmse_UGV2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('UGV1','UGV2');

disp(['Average UGV1 Position Estimation Error: ',num2str(mean(rmse_UGV1))]);
disp(['Average UGV2 Position Estimation Error: ',num2str(mean(rmse_UGV2))]);

%% Landmark Position Error
error_landmark1 = pos_landmark1-real_landmarks;
error_landmark2 = pos_landmark2-real_landmarks;
error_landmark1 = reshape(error_landmark1(:),[2*num_landmarks,timesteps]);
error_landmark2 = reshape(error_landmark2(:),[2*num_landmarks,timesteps]);
rmse_landmark1 = sqrt(sum(error_landmark1.^2,1));
rmse_landmark2 = sqrt(sum(error_landmark2.^2,1));

subplot(412); hold on; 
title('Landmark Estimation Error');
plot(time,rmse_landmark1,'r');
plot(time,rmse_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('UGV1','UGV2');


disp(['Average Landmark Estimation Error from UGV1: ',num2str(mean(rmse_landmark1))]);
disp(['Average Landmark Estimation Error from UGV2: ',num2str(mean(rmse_landmark2))]);

if fusion_flag>0
    error_landmark_SF = pos_landmark_SF-real_landmarks;
    error_landmark_SF = reshape(error_landmark_SF(:),[2*num_landmarks,timesteps]);
    rmse_landmark_SF = sqrt(sum(error_landmark_SF.^2,1));
    plot(time,rmse_landmark_SF,'g');
    legend('UGV1','UGV2','Fusion');
    disp(['Average Landmark Estimation Error from Fusion: ',num2str(mean(rmse_landmark_SF))]);
end

%% Covariance Matrix
cov_landmark1 = reshape(cov_landmark1(:),[4*num_landmarks,timesteps]);
cov_landmark2 = reshape(cov_landmark2(:),[4*num_landmarks,timesteps]);
sum_cov_landmark1 = sqrt(sum(cov_landmark1.^2,1));
sum_cov_landmark2 = sqrt(sum(cov_landmark2.^2,1));

subplot(413); hold on; 
title('Landmark Covariance');
plot(time,sum_cov_landmark1,'r');
plot(time,sum_cov_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('Covariance [m^2]'); legend('UGV1','UGV2');

if fusion_flag>0
    cov_landmark_SF = reshape(cov_landmark_SF(:),[4*num_landmarks,timesteps]);
    sum_cov_landmark_SF = sqrt(sum(cov_landmark_SF.^2,1));
    plot(time,sum_cov_landmark_SF,'g');
    legend('UGV1','UGV2','Fusion');
end

%% Number of Landmarks within FOV
subplot(414); hold on;
title('Number of landmarks within FOV');
plot(time,squeeze(sum(index_fov(1,:,:),2)),'r');
plot(time,squeeze(sum(index_fov(2,:,:),2)),'b');
grid on; xlabel('Time (s)'); ylabel('No. of landmarks'); legend('UGV1','UGV2');

if fusion_flag>0
    plot(time,squeeze(sum(index_fov(3,:,:),2)),'g');
    legend('UGV1','UGV2','Fusion');
end