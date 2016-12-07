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

num=100;

behavior_sequence = cell(num,1);
cost = cell(num,1);
time_to_plan_end = cell(num,1);

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
%% Behaviors
%behaviors = {@flocking, @flock_east, @flock_north, @flock_west, @flock_south, @rendezvous, @antirendezvous, @move_stop, @line_x, @line_y};
%behaviors = {@rendezvous, @antirendezvous, @flocking, @move_right, @move_up, @move_left, @move_down, @move_stop};
behaviors = {@antirendezvous, @flocking, @flock_east, @flock_north, @flock_west, @flock_south, @line_x, @line_y};
%behaviors = {@antirendezvous, @rendezvous, @line_x, @line_y, @flocking};

% %% Plan AStar
% for i = 1:num
%     i
%     time_to_plan_start = tic;
%     [behavior_sequence{i}, cost{i}] = plan_behaviors(map, target_coverage, robot_radius, ...
%         inflated_obstacles, behaviors, poses(:,:,1), ti, tf, dt, dT);
%     time_to_plan_end{i} = toc(time_to_plan_start);
%     %cost
% end
    
%% Plan IMHAStar
parfor i = 1:num
    %% Robots
    i
    robot = zeros(N,3);
    robot(:,1) = initial_box.bottom_left_x + initial_box.width*rand(N,1);
    robot(:,2) = initial_box.bottom_left_y + initial_box.height*rand(N,1);
    robot(:,3) = wrapToPi(2*pi*rand(N,1));
    labels = cellstr(num2str((1:N)'));
    
    ts = round((tf-ti)/dt) + 1;
    poses = zeros(N,3,ts);
    poses(:,:,1) = robot;
    
    %% Grid
    map_grid = 0.5*ones([map.grid_y map.grid_x ts]);
    
    %% Plan
    time_to_plan_start = tic;
    [behavior_sequence{i}, cost{i}] = plan_behaviors_IMHAStar(map, target_coverage, robot_radius, ...
        inflated_obstacles, behaviors, poses(:,:,1), ti, tf, dt, dT);
    time_to_plan_end{i} = toc(time_to_plan_start);
    %cost
end