function particles_SF = Fusion_covariance(particles_UGV1, particles_UGV2, particles_SF, index_fov)
global num_landmarks num_particles 
for l = 1:num_landmarks
    for p = 1:num_particles
        if index_fov(3,l) % seen by both UGVs    
            a = particles_UGV1(p).landmarks(l).pos;
            b = particles_UGV2(p).landmarks(l).pos;
            inv_A = inv(particles_UGV1(p).landmarks(l).P);
            inv_B = inv(particles_UGV2(p).landmarks(l).P);

            covariance_det = @(omega) 1/det(omega*inv_A+(1-omega)*inv_B);
            omega = fminbnd(covariance_det,0,1);
%             omega = 0.5;
            particles_SF(p).landmarks(l).P = inv(omega*inv_A+(1-omega)*inv_B);
            particles_SF(p).landmarks(l).pos = particles_SF(p).landmarks(l).P*(omega*inv_A*a+(1-omega)*inv_B*b);     
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
