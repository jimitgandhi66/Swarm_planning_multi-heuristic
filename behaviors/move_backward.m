%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Move Backward
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = move_backward(poses_in, dt)
    linear_velocity = -1.0;

    assert(size(poses_in,2) == 3);
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    for i=1:N
        u_v = linear_velocity;
        u_w = 0.0;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

