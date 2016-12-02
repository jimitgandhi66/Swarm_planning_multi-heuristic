%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Plan Behaviors
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [best_sequence] = plan_behaviors(target_center, target_radius, ...
    robot_radius, obstacles, behaviors, initial_poses, ...
    ti, tf, dt, dT, mapsize)

    target_center = target_center(:);
    assert(numel(target_center) == 2);
    assert(ismatrix(initial_poses));
    assert(size(initial_poses,2) == 3);

    N = size(initial_poses,1);
    B = numel(behaviors);
    H = 20.0;

    best_cost = Inf;
    best_sequence = [];
    best_poses = [];
    make_sequence_astar();
    assert(best_cost < Inf);

    function make_sequence_astar()
        q_data{1} = {0, [], initial_poses};
        q_priority(1) = sum(pdist2(initial_poses(:,1:2),target_center'));
        while ~isempty(q_priority)
            if best_cost < q_priority(end), return; end
            q_priority(end) = [];
            data = q_data{end}; q_data(end) = [];
            cost = data{1};
            sequence = data{2};
            poses = data{3};
            tbegin = ti + numel(sequence)*dT;
            tend = min([tbegin+dT-dt tf]);
            if tbegin>tf
                sequence
                if target_reached(poses) && (cost < best_cost)
                    best_cost = cost;
                    best_sequence = sequence
                    best_poses = poses;
                    I = (q_priority < best_cost);
                    q_priority = q_priority(I);
                    q_data = q_data(I);
                end
                continue;
            end
            priorities = -1*ones(1,B);
            datas = cell(1,B);
            parfor b=1:B
                behavior = behaviors{b};
                sequence_next = [sequence; b];
                poses_next = poses;
                cost_next = cost;
                valid_sequence = true;
                for t=tbegin:dt:tend
                    poses_old = poses_next;
                    poses_next = behavior(poses_next, dt);
                    if ~valid_poses(poses_next, obstacles, mapsize) 
                        valid_sequence = false;
                        break;
                    end
                    poses_delta = poses_next(:,1:2)' - poses_old(:,1:2)';
                    cost_next = cost_next + sum(sqrt(sum(poses_delta.^2,1)));
                end
                if ~valid_sequence, continue; end
                cost_to_go = ...
                    sum(max(pdist2(poses_next(:,1:2),target_center') ...
                        - (target_radius-robot_radius),0));
                cost_estimate = cost_next + H*cost_to_go;
                if cost_estimate < best_cost
                    priorities(b) = cost_estimate; 
                    datas{b} = {cost_next, sequence_next, poses_next};
                end
            end
            indices = (priorities >= 0);
            if ~isempty(indices)
                q_priority = [q_priority priorities(indices)];
                q_data = [q_data datas(indices)];
                [~,I] = sort(q_priority, 'descend');
                q_priority = q_priority(I);
                q_data = q_data(I);
            end
        end
    end
    
    function make_sequence_dfs(poses, sequence, cost)
        tbegin = ti + numel(sequence)*dT;
        tend = min([tbegin+dT tf]);
        if tbegin>tf
            if target_reached(poses) && (cost < best_cost)
                best_cost = cost;
                best_sequence = sequence;
            end
            return;
        end
        for b=1:B
            behavior = behaviors{b};
            poses_next = poses;
            valid_sequence = true;
            for t=tbegin:dt:tend
                poses_next = behavior(poses_next, dt);
                if ~valid_poses(poses_next) 
                    valid_sequence = false;
                    break;
                end
            end
            if ~valid_sequence, continue; end
            sequence_next = [sequence; b];
            cost_next = cost;
            for i=1:N
                cost_next = cost_next + ...
                            norm(poses_next(i,1:2)' - poses(i,1:2)',2);
            end
            cost_to_go = -(target_radius-robot_radius)*N + ...
                         sum(pdist2(poses_next(:,1:2),target_center'));
            cost_estimate = (1-alpha)*cost_next + alpha*cost_to_go;
            if cost_estimate < best_cost
                make_sequence_dfs(poses_next, sequence_next, cost_next);
            end
        end
    end

    function r = target_reached(poses)
        for i=1:size(poses,1)
            d = norm(poses(i,1:2)'-target_center,2); 
            if d >= target_radius-robot_radius
                r = false;
                return;
            end
        end
        r = true;
    end

end

function r = valid_poses(poses, obstacles, mapsize)
    if any(poses(:,1) > mapsize) || ...
       any(poses(:,2) > mapsize) || ...
       any(poses(:,1) < -mapsize) || ...
       any(poses(:,2) < -mapsize)
        r = false;
        return;
    end
    for i=1:numel(obstacles)
        obs = obstacles{i};
        if any(inpolygon(poses(:,1),poses(:,2),obs(:,1),obs(:,2)))
            r = false;
            return;
        end
    end
    r = true;
end


