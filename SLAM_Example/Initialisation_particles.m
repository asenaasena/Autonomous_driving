function particles = Initialisation_particles(pos_UGV,pos_landmarks)
global num_particles num_landmarks P0
for p = 1:num_particles
    particles(p).w = 1/num_particles;           % weight
    particles(p).position = pos_UGV + [sqrt(P0)*randn(2,1);0];   % UGV position
    for l=1:num_landmarks
        particles(p).landmarks(l).pos = pos_landmarks(:,l); % Landmark position
        particles(p).landmarks(l).P = P0;
    end
end