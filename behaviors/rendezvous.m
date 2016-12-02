%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Rendezvous
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = rendezvous(poses_in, dt)
    connectivity_radius = 20.0;
    gain_v = 1.0;
    gain_w = 1.0;

    assert(size(poses_in,2) == 3);
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    for i=1:N
        position_i = poses_in(i, 1:2)';
        heading_i = poses_in(i, 3);
        
        v = zeros(2, 1);
        w = 0;
        n = 0;
        for j=1:N
            if i==j, continue; end
            position_j = poses_in(j, 1:2)';
            heading_j = poses_in(j, 3);
            
            d = position_j - position_i;
            if norm(d,2) < connectivity_radius
                dv = d;
                n = n + 1;
            else 
                dv = 0;
            end
            v = v + dv;
        end
        v = v / (n + 1);
        dtheta = atan2(v(2), v(1)) - heading_i;
        w = atan2(sin(dtheta), cos(dtheta));
        
        b = [cos(heading_i); sin(heading_i)];
        u_v = gain_v * v' * b;
        u_w = gain_w * w;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

