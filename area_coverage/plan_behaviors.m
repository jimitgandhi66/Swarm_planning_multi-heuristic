%% Swarm Switching Behaviors 
% Description : Switching Behaviors - Plan Behaviors
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

function [best_sequence, best_cost] = plan_behaviors(map, target_coverage, ...
    robot_radius, obstacles, behaviors, initial_poses, ...
    ti, tf, dt, dT)

    assert((map.size_x / map.grid_x) == (map.size_y / map.grid_y));
    assert(ismatrix(initial_poses));
    assert(size(initial_poses,2) == 3);

    N = size(initial_poses,1);
    B = numel(behaviors);
    H = 10.0;

    best_cost = Inf;
    best_sequence = [];
    best_poses = [];
    make_sequence_astar();
    assert(best_cost < Inf);

    function make_sequence_astar()
        initial_unseen = ones(map.grid_y, map.grid_x);
        q_data{1} = {0, [], initial_poses, initial_unseen};
        q_priority(1) = heuristic_cost_to_go(...
            initial_unseen, initial_poses, target_coverage, map);
        while ~isempty(q_priority)
            if best_cost<=q_priority(end), return; end
            q_priority(end) = [];
            data = q_data{end}; q_data(end) = [];
            cost = data{1};
            sequence = data{2};
            poses = data{3};
            unseen = data{4};
            tbegin = ti + numel(sequence)*dT;
            tend = min([tbegin+dT-dt tf]);
            if tbegin>tf
                %sequence
                %coverage_ratio(unseen)
                if (coverage_ratio(unseen) >= target_coverage) && (cost < best_cost)
                    best_cost = cost;
                    best_sequence = sequence;
                    best_poses = poses;
                    I = (q_priority < best_cost);
                    q_priority = q_priority(I);
                    q_data = q_data(I);
                    %best_cost
                    %final_cost = best_cost(1);
                end
                continue;
            end
            priorities = -1*ones(1,B);
            datas = cell(1,B);
            parfor b=1:B
                behavior = behaviors{b};
                %sequence
                %b
                sequence_next = [sequence; b];
                poses_next = poses;
                unseen_next = unseen;
                cost_next = cost;
                valid_sequence = true;
                for t=tbegin:dt:tend
                    poses_old = poses_next;
                    poses_next = behavior(poses_next, dt);
                    if ~valid_poses(poses_next, obstacles, map) 
                        valid_sequence = false;
                        break;
                    end
                    poses_delta = poses_next(:,1:2)' - poses_old(:,1:2)';
                    cost_next = cost_next + sum(sqrt(sum(poses_delta.^2,1)));
                    unseen_next = fill_unseen(unseen_next,poses_next,map);
                end
                if ~valid_sequence, continue; end
                cost_next = cost_next + sum(unseen_next(:))/length(unseen_next(:));
                cost_to_go = heuristic_cost_to_go(unseen_next, poses, ...
                    target_coverage, map);
                cost_estimate = cost_next + H*cost_to_go;
                if cost_estimate < best_cost
                    priorities(b) = cost_estimate; 
                    datas{b} = {cost_next,sequence_next,poses_next,unseen_next};
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
    
    function r = coverage_ratio(unseen)
        r = sum(1-unseen(:)) / length(unseen(:));
    end

end

function c = heuristic_cost_to_go(unseen, poses, target, map)
    N = size(poses,1);
    c = (sqrt(2) / 3) * (map.size_x / map.grid_x) * ... 
        max(0, target*length(unseen(:)) - sum(1-unseen(:)) - 4*N);
end

function unseen = fill_unseen(unseen, poses, map)
    %assert(all(poses(:,1) >= 0));
    %assert(all(poses(:,2) >= 0));
    %assert(all(poses(:,1) <= map.size_x));
    %assert(all(poses(:,2) <= map.size_y));
    grid_x = round(poses(:,1)*map.grid_x / map.size_x) + 1;
    grid_y = round(poses(:,2)*map.grid_y / map.size_y) + 1;
    i = (grid_x<=map.grid_x)&(grid_x>=1)&(grid_y<=map.grid_y)&(grid_y>=1);
    unseen(sub2ind(size(unseen),grid_y(i),grid_x(i))) = 0;
end

function r = valid_poses(poses, obstacles, map)
    if any(poses(:,1) >= map.size_x) || any(poses(:,1) < 0) || ...
       any(poses(:,2) >= map.size_y) || any(poses(:,2) < 0)
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


