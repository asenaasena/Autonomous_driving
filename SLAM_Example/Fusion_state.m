function particles_SF = Fusion_state(particles_UGV1, particles_UGV2, particles_SF, index_fov)
global num_landmarks num_particles
for l = 1:num_landmarks     
    for p = 1:num_particles                   
        if index_fov(3,l) % seen by both UGVs
            a = particles_UGV1(p).landmarks(l).pos;
            b = particles_UGV2(p).landmarks(l).pos;
            A = particles_UGV1(p).landmarks(l).P;
            B = particles_UGV2(p).landmarks(l).P;
            
            particles_SF(p).landmarks(l).pos = a+A/(A+B)*(b-a);
            particles_SF(p).landmarks(l).P   = A-A/(A+B)*A';
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