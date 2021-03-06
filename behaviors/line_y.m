%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Line Y
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = line_y(poses_in, dt)
    assert(size(poses_in,2) == 3);
    N = size(poses_in, 1);
    formation = zeros(N,2);
    formation(1:N,2) = (1:N)*2;
    
    poses_out = formation_control(poses_in, formation, dt);
end

