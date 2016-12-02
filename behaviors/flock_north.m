%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Flock North
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = flock_north(poses_in, dt)
    direction = [0; 1];
    gain = 1.0;
    
    bias = gain*direction;
    
    poses_out = flock_biased(poses_in, bias, dt);
end

