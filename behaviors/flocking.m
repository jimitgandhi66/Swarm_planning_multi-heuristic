%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Flocking
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = flocking(poses_in, dt)
    bias = [0; 0];
    poses_out = flock_biased(poses_in, bias, dt);
end

