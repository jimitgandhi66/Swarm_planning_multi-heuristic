%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Align Heading
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = align_heading(poses_in, dt)
    connectivity_radius = 100.0;
    gain_v = 1.0;
    gain_w = 2.0;

    assert(size(poses_in,2) == 3);
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    for i=1:N
        position_i = poses_in(i, 1:2)';
        heading_i = poses_in(i, 3);
        
        w = 0;
        n = 0;
        for j=1:N
            if i==j, continue; end
            position_j = poses_in(j, 1:2)';
            heading_j = poses_in(j, 3);
            
            d = position_j - position_i;
            if norm(d,2) < connectivity_radius
                dtheta = heading_j - heading_i;
                w = w + atan2(sin(dtheta), cos(dtheta));
                n = n + 1;
            end
        end
        if n > 0
            w = wrapToPi(dtheta / n);
        end
        
        u_v = 0.0;
        u_w = gain_w * w;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

