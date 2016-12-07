%% Swarm Switching Behaviors for Motion
% Description : Switching Behaviors - Motion
% Author      : Sasanka Nagavalli 
% Date        : February 5, 2016
% Other Files :

%% Clean up
clear;
clc;
clf;
close all;

%% Paths
addpath('../behaviors');
data_folder = '../data';

%% Simulation parameters
N = 10;
ti = 0;
dt = 0.1;
tf = 100;
dT = 10;
map.size_y = 100;
map.size_x = 100;
map.grid_y = 100;
map.grid_x = 100;

target_coverage = 0.1;
robot_radius = 0.5;
O = 0;
obstacle_max_sides = 8;
obstacle_min_radius = 5;
obstacle_max_radius = 7;
initial_box.bottom_left_x = map.size_x/2 - map.size_x/8; 
initial_box.bottom_left_y = map.size_y/2 - map.size_y/8; 
initial_box.width = map.size_x/8;
initial_box.height = map.size_y/8;

init_poses_x = [0.696266337082995;0.093820026774866;0.525404403859336;0.530344218392863;0.861139811393332;0.484853333552102;0.393456361215266;0.671431139674026;0.741257943454207;0.520052467390387];  
init_poses_y = [0.404579995857626;0.448372912066495;0.365816176838171;0.763504640848813;0.627896379614169;0.771980385554245;0.932853570278820;0.972740854003014;0.192028349427775;0.138874202829155];
init_poses_th = [0.347712671277525;0.149997253831683;0.586092067231462;0.262145317727807;0.044454092278238;0.754933267231179;0.242785357820962;0.442402313001943;0.687796085120107;0.359228210401861];
%% Robots
robot = zeros(N,3);
robot(:,1) = initial_box.bottom_left_x + initial_box.width*init_poses_x;
robot(:,2) = initial_box.bottom_left_y + initial_box.height*init_poses_y;
robot(:,3) = wrapToPi(2*pi*init_poses_th);
labels = cellstr(num2str((1:N)'));

ts = round((tf-ti)/dt) + 1;
poses = zeros(N,3,ts);
poses(:,:,1) = robot;

%% Grid
map_grid = 0.5*ones([map.grid_y map.grid_x ts]);

%% Obstacles
obstacles = {};
inflated_obstacles = {};
for i=1:O
    while true
        iradius = (obstacle_max_radius-obstacle_min_radius)*rand(1,1);
        iradius = obstacle_min_radius + iradius;
        oradius = iradius + robot_radius;
        cx = map.size_x*rand(1,1)*0.8;
        cy = map.size_y*rand(1,1)*0.8;
        angles = linspace(0,2.*pi,randi([4 obstacle_max_sides+1],1,1))';
        obs = [cx+iradius*cos(angles) cy+iradius*sin(angles)];
        iobs = [cx+oradius*cos(angles) cy+oradius*sin(angles)];
        if ~(any(iobs(:,1) > map.size_x) || any(iobs(:,1) < 0) || ...
             any(iobs(:,2) > map.size_y) || any(iobs(:,2) < 0) || ...
             any(inpolygon(poses(:,1,1),poses(:,2,1),obs(:,1),obs(:,2))))
            break;
        end
    end
    obstacles{i} = obs;
    inflated_obstacles{i} = iobs;
end
clf;
axis([0 map.size_x 0 map.size_y]), hold on;
for i=1:O
    obs = obstacles{i};
    fill(obs(:,1),obs(:,2),'b');
end
drawnow;

%% Behaviors
%behaviors = {@flocking, @flock_east, @flock_north, @flock_west, @flock_south, @rendezvous, @antirendezvous, @move_stop, @line_x, @line_y};
%behaviors = {@rendezvous, @antirendezvous, @flocking, @move_right, @move_up, @move_left, @move_down, @move_stop};
behaviors = {@antirendezvous, @flocking, @flock_east, @flock_north, @flock_west, @flock_south, @line_x, @line_y};
%behaviors = {@antirendezvous, @rendezvous, @line_x, @line_y, @flocking};

%% Plan
time_to_plan_start = tic;
[behavior_sequence, cost] = plan_behaviors(map, target_coverage, robot_radius, ...
   inflated_obstacles, behaviors, poses(:,:,1), ti, tf, dt, dT);
time_to_plan_end = toc(time_to_plan_start)

%% Execute
figure;
ts = 0;
for Ts=1:length(behavior_sequence)
    tbegin = ti + (Ts-1)*dT;
    tend = min([tbegin+dT-dt tf]);
    
    for t=tbegin:dt:tend
        % Time step
        ts = ts + 1;

        % Update
        behavior = behaviors{behavior_sequence(Ts)};
        poses(:,:,ts+1) = behavior(poses(:,:,ts),dt);

        map_grid(:,:,ts+1) = map_grid(:,:,ts);
        grid_x = round(poses(:,1,ts+1)*map.grid_x / map.size_x) + 1;
        grid_y = round(poses(:,2,ts+1)*map.grid_y / map.size_y) + 1;
        i = (grid_x<=map.grid_x)&(grid_x>=1)&(grid_y<=map.grid_y)&(grid_y>=1);
        grid_x = grid_x(i);
        grid_y = grid_y(i);
        i = sub2ind(size(map_grid), grid_y, grid_x, ones(size(grid_y))*(ts+1));
        map_grid(i) = 1;

        % Plot
        clf;
        axis([0 map.size_x 0 map.size_y]), hold on;
        image([0 map.size_x],[0 map.size_y],repmat(map_grid(:,:,ts),1,1,3));
        colormap('gray');
        for i=1:O
            obs = obstacles{i};
            fill(obs(:,1),obs(:,2),'g');
        end
        for i=1:N
            plot(poses(i,1,ts)+robot_radius*cos(0:0.01*pi:2*pi),...
                 poses(i,2,ts)+robot_radius*sin(0:0.01*pi:2*pi));
            plot([poses(i,1,ts) poses(i,1,ts)+robot_radius*cos(poses(i,3,ts))],...
                 [poses(i,2,ts) poses(i,2,ts)+robot_radius*sin(poses(i,3,ts))]);
            % text(poses(i,1,ts),poses(i,2,ts),labels(i),...
            %     'VerticalAlignment','bottom','HorizontalAlignment','right');
        end
        drawnow;
    end
end

%% Save data
trial_id = ['trial_',datestr(now,30)];
folder = [data_folder,'/',trial_id];
mkdir(folder);
save([folder,'/',...
      'N_',num2str(N),'_',...
      'tf_',strrep(num2str(tf,'%010.4f'),'.','d'),'_',...
      'dt_',strrep(num2str(dt,'%010.4f'),'.','d'),'_',...
      'dT_',strrep(num2str(dT,'%010.4f'),'.','d')]);
  