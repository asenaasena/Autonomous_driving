function particles_SF = Fusion_measure(particles_UGV1, particles_UGV2, particles_SF, z1, z2, index_fov)
global num_landmarks num_particles Rm
for l = 1:num_landmarks
    for p = 1:num_particles   
        if index_fov(3,l) % seen by both UGVs         
            z_p_1 = Measurement(particles_UGV1(p).position,particles_UGV1(p).landmarks(l).pos);
            z_p_2 = Measurement(particles_UGV2(p).position,particles_UGV2(p).landmarks(l).pos);
            residual = [z1(:,l);z2(:,l)] - [z_p_1;z_p_2];

            % Calculate the Kalman gain
            H = [eye(2);eye(2)];
            P = particles_SF(p).landmarks(l).P;
            S = H*P*H' + Rm;
            K = P*H'/S;

            % Update fused estimates
            particles_SF(p).landmarks(l).pos = particles_SF(p).landmarks(l).pos + K*residual;
            particles_SF(p).landmarks(l).P = (eye(2)-K*H)*P;
            continue;
        end
        if index_fov(1,l) % seen by UGV1 only
            particles_SF(p).landmarks(l).pos = particles_UGV1(p).landmarks(l).pos;
            particles_SF(p).landmarks(l).P   = particles_UGV1(p).landmarks(l).P;
        end
        if index_fov(2,l) % seen by UGV2 only
            particles_SF(p).landmarks(l).pos = particles_UGV2(p).landmarks(l).pos;
            particles_SF(p).landmarks(l).P   = particles_UGV2(p).landmarks(l).P;
        end
    end  
end

