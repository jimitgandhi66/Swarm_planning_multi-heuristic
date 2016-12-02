%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Turn Counterclockwise
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = turn_counterclockwise(poses_in, dt)
    angular_velocity = pi/8;

    assert(size(poses_in,2) == 3);
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    for i=1:N
        u_v = 0.0;
        u_w = angular_velocity;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

