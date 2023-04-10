clc; clear all; close all; warning off;
%% Simulation Settings
% Flag for sensor fusion
fusion_flag = 0;            % 0: no fusion / 1: state fusion / 2: measurement fusion / 3: covariance intersection

rng(2023)
Initialisation_scenario;

% Initialise particles
particles_UGV1 = Initialisation_particles(initial_UGV1,initial_landmarks);
particles_UGV2 = Initialisation_particles(initial_UGV2,initial_landmarks);
particles_SF   = Initialisation_particles(initial_UGV1,initial_landmarks);

% Initialise for recording
pos_UGV1 = zeros(3,timesteps);
pos_UGV2 = zeros(3,timesteps);
pos_landmark1 = zeros(2,num_landmarks,timesteps);
pos_landmark2 = zeros(2,num_landmarks,timesteps);
pos_landmark_SF = zeros(2,num_landmarks,timesteps);
cov_landmark1 = zeros(2,2,num_landmarks,timesteps);
cov_landmark2 = zeros(2,2,num_landmarks,timesteps);
cov_landmark_SF = zeros(2,2,num_landmarks,timesteps);

pos_UGV1(:,1) = initial_UGV1;
pos_UGV2(:,1) = initial_UGV2;
pos_landmark1(:,:,1) = initial_landmarks;
pos_landmark2(:,:,1) = initial_landmarks;
pos_landmark_SF(:,:,1) = initial_landmarks;
for l = 1:num_landmarks
    cov_landmark1(:,:,l,1) = P0;
    cov_landmark2(:,:,l,1) = P0;
    cov_landmark_SF(:,:,l,1) = P0;
end


%% Simulation
for t = 2:timesteps    
    %% Run particle filter SLAM
    particles_UGV1 = SLAM_PF(particles_UGV1,meas_landmark1(:,:,t),index_fov(1,:,t));
    particles_UGV2 = SLAM_PF(particles_UGV2,meas_landmark2(:,:,t),index_fov(2,:,t));
    
    %% Fusion
    switch fusion_flag
        case 1 % State fusion
            particles_SF = Fusion_state(particles_UGV1, particles_UGV2, particles_SF, index_fov(:,:,t));
        case 2 % Measurement fusion
            particles_SF = Fusion_measure(particles_UGV1, particles_UGV2, particles_SF, meas_landmark1(:,:,t), meas_landmark2(:,:,t), index_fov(:,:,t));
        case 3 % Covariance Intersection
            particles_SF = Fusion_covariance(particles_UGV1, particles_UGV2, particles_SF, index_fov(:,:,t));
        otherwise
    end

    %% Save
    for p = 1:num_particles
        pos_particles1(:,p) = particles_UGV1(p).position;
        pos_particles2(:,p) = particles_UGV2(p).position;
        for l = 1:num_landmarks    
            pos_landmark1(:,l,t) = pos_landmark1(:,l,t)+particles_UGV1(p).landmarks(l).pos/num_particles;
            pos_landmark2(:,l,t) = pos_landmark2(:,l,t)+particles_UGV2(p).landmarks(l).pos/num_particles;
            pos_landmark_SF(:,l,t) = pos_landmark_SF(:,l,t)+particles_SF(p).landmarks(l).pos/num_particles;
            cov_landmark1(:,:,l,t) = cov_landmark1(:,:,l,t)+particles_UGV1(p).landmarks(l).P/num_particles;
            cov_landmark2(:,:,l,t) = cov_landmark2(:,:,l,t)+particles_UGV2(p).landmarks(l).P/num_particles;
            cov_landmark_SF(:,:,l,t) = cov_landmark_SF(:,:,l,t)+particles_SF(p).landmarks(l).P/num_particles;
        end
    end
    pos_UGV1(:,t) = mean(pos_particles1,2);
    pos_UGV2(:,t) = mean(pos_particles2,2);

    %% Plot
    %Plot_animation;
    drawnow;

end
Plot_analysis;