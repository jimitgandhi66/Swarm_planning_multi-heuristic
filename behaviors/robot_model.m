%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Robot Model
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [pose_out] = robot_model(pose_in, u_v, u_w, dt)
    max_linear_velocity = 2.0;
    max_angular_velocity = pi/8;
    
    assert(numel(pose_in) == 3);
    
    px_i = pose_in(1);
    py_i = pose_in(2);
    heading_i = pose_in(3);

    u_v = max(-max_linear_velocity,min(max_linear_velocity,u_v));
    u_w = max(-max_angular_velocity,min(max_angular_velocity,u_w));

    pose_out = [px_i + u_v*cos(heading_i)*dt;
                py_i + u_v*sin(heading_i)*dt;
                wrapToPi(heading_i + u_w*dt)];
end
