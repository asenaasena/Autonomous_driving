clf;
hold on;
% Plot the landmarks
for l=1:num_landmarks
    fig(1) = plot(real_landmarks(1,l),real_landmarks(2,l),'b*');
    if index_seen(1,l,t)
        fig(2) = plot(pos_landmark1(1,l,t),pos_landmark1(2,l,t),'ko');
    end
    if index_seen(2,l,t)
        plot(pos_landmark2(1,l,t),pos_landmark2(2,l,t),'ko');
    end
    if fusion_flag>0 && index_seen(3,l,t)
        fig(6) = plot(pos_landmark_SF(1,l,t),pos_landmark_SF(2,l,t),'go');
    end
end

% Plot the UGVs
fig(3) = plot(real_UGV1(1,1:t),real_UGV1(2,1:t),'r');
text(real_UGV1(1,t),real_UGV1(2,t),'UGV1');
plot(real_UGV2(1,1:t),real_UGV2(2,1:t),'r');
text(real_UGV2(1,t),real_UGV2(2,t),'UGV2');

% Plot the particles
fig(4) = plot(pos_particles1(1,:),pos_particles1(2,:),'r.');
plot(pos_particles2(1,:), pos_particles2(2,:),'r.');
fig(5) = plot(pos_UGV1(1,t), pos_UGV1(2,t),'ro','MarkerSize',15);
plot(pos_UGV2(1,t), pos_UGV2(2,t),'ro','MarkerSize',15);

% Show the sensor measurement as an arrow
for l=1:num_landmarks
    if index_fov(1,l,t)
        plot([real_UGV1(1,t), real_UGV1(1,t)+meas_landmark1(1,l,t)* cos(meas_landmark1(2,l,t))], ...
             [real_UGV1(2,t), real_UGV1(2,t)+meas_landmark1(1,l,t)* sin(meas_landmark1(2,l,t))],'Color',0.5*ones(1,3));
    end
    if index_fov(2,l,t)
        plot([real_UGV2(1,t), real_UGV2(1,t)+meas_landmark2(1,l,t)* cos(meas_landmark2(2,l,t))], ...
             [real_UGV2(2,t), real_UGV2(2,t)+meas_landmark2(1,l,t)* sin(meas_landmark2(2,l,t))],'Color',0.5*ones(1,3));
    end
end

% Show the sensor's field of range
loop = 1:11;
circ = max_read_distance*[cosd(36*loop);sind(36*loop)];
plot(real_UGV1(1,t)+circ(1,:),real_UGV1(2,t)+circ(2,:),'--','Color',0.5*ones(1,3));
plot(real_UGV2(1,t)+circ(1,:),real_UGV2(2,t)+circ(2,:),'--','Color',0.5*ones(1,3));

if size(fig,2)>5
    legend(fig,'True Landmark','Estimated Landmark','True Position','Particles','Mean of Particles','Fused Landmark')
else
    legend(fig,'True Landmark','Estimated Landmark','True Position','Particles','Mean of Particles')
end

grid on;
xlabel('x (m)')
ylabel('y (m)')
axis([-30, 30, -30, 30]);