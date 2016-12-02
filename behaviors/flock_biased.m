%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Flock with Biased Direction
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [poses_out] = flock_biased(poses_in, bias, dt)
    min_linear_velocity = 1.0;
    repulsion_radius = 5.0;
    alignment_radius = 10.0;
    attraction_radius = 20.0;
    gain_v = 1.0;
    gain_w = 1.0;

    assert(size(poses_in,2) == 3);
    N = size(poses_in, 1);
    bias = bias(:);
    
    poses_out = poses_in;
    for i=1:N
        position_i = poses_in(i, 1:2)';
        heading_i = poses_in(i, 3);
        
        v = 0;        
        w = 0;
        n = 0;
        for j=1:N
            if i==j, continue; end
            position_j = poses_in(j, 1:2)';
            heading_j = poses_in(j, 3);
            
            n = n + 1;
            d = position_j - position_i;
            if norm(d,2) < repulsion_radius
                dv = - d / (d'*d);
                dtheta = atan2(dv(2), dv(1)) - heading_i;
            elseif norm(d,2) < alignment_radius
                dv = 0;
                dtheta = heading_j - heading_i;
            elseif norm(d,2) < attraction_radius
                dv = d;
                dtheta = atan2(dv(2), dv(1)) - heading_i;
            else 
                dv = zeros(2, 1);
                dtheta = 0;
                n = n - 1;
            end 
            v = v + dv;
            w = w + atan2(sin(dtheta), cos(dtheta));
        end
        v = v / (n + 1);
        w = wrapToPi(w / (n + 1));
        
        v = v + bias;
        dtheta = atan2(bias(2), bias(1)) - heading_i;
        w = w + atan2(sin(dtheta), cos(dtheta));
        
        b = [cos(heading_i); sin(heading_i)];
        u_v = gain_v * v' * b;
        u_w = gain_w * w;
        
        u_v = max(min_linear_velocity,u_v);
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

