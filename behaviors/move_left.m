%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Move Left
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = move_left(poses_in, dt)
    gain_v = 1.0;
    gain_w = 1.0;

    assert(size(poses_in,2) == 3);
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    for i=1:N
        position_i = poses_in(i, 1:2)';
        heading_i = poses_in(i, 3);
        
        v = [-1; 0];
        dtheta = atan2(v(2), v(1)) - heading_i;
        w = atan2(sin(dtheta), cos(dtheta));
        
        b = [cos(heading_i); sin(heading_i)];
        u_v = gain_v * v' * b;
        u_w = gain_w * w;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

