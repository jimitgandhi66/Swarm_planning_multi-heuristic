%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Flock East
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = flock_east(poses_in, dt)
    direction = [1; 0];
    gain = 1.0;
    
    bias = gain*direction;
    
    poses_out = flock_biased(poses_in, bias, dt);
end

