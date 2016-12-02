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
mapsize = 20;
ti = 0;
dt = 0.1;
tf = 50;
dT = 5;
target_center = [10 10]';
target_radius = 7;
robot_radius = 0.5;
O = 2;
obstacle_max_sides = 8;
obstacle_min_radius = 2;
obstacle_max_radius = 5;
initial_box.bottom_left_x = -mapsize+1; 
initial_box.bottom_left_y = -mapsize+1; 
initial_box.width = target_radius*2; 
initial_box.height = target_radius*2; 

%% Robots
robot = zeros(N,3);
robot(:,1) = initial_box.bottom_left_x + initial_box.width*rand(N,1);
robot(:,2) = initial_box.bottom_left_y + initial_box.height*rand(N,1);
robot(:,3) = wrapToPi(rand(N,1));
labels = cellstr(num2str((1:N)'));

ts = round((tf-ti)/dt) + 1;
poses = zeros(N,3,ts);
poses(:,:,1) = robot;

%% Obstacles
obstacles = {};
inflated_obstacles = {};
for i=1:O
    while true
        iradius = (obstacle_max_radius-obstacle_min_radius)*rand(1,1);
        iradius = obstacle_min_radius + iradius;
        oradius = iradius + robot_radius;
        cx = (-mapsize + 2*mapsize*rand(1,1))*0.8;
        cy = (-mapsize + 2*mapsize*rand(1,1))*0.8;
        angles = linspace(0,2.*pi,randi([4 obstacle_max_sides+1],1,1))';
        obs = [cx+iradius*cos(angles) cy+iradius*sin(angles)];
        if (norm([cx; cy]-target_center,2) >= target_radius+iradius) && ...
           ~any(inpolygon(poses(:,1,1),poses(:,2,1),obs(:,1),obs(:,2)))
            break;
        end
    end
    obstacles{i} = obs;
    inflated_obstacles{i} = [cx+oradius*cos(angles) cy+oradius*sin(angles)];
end
clf;
axis([-mapsize mapsize -mapsize mapsize]), hold on;
plot(target_center(1)+target_radius*cos(0:0.01*pi:2*pi),...
     target_center(2)+target_radius*sin(0:0.01*pi:2*pi));
for i=1:O
    obs = obstacles{i};
    fill(obs(:,1),obs(:,2),'b');
end
drawnow;

%% Behaviors
behaviors = {@rendezvous, @flocking, @flock_east, @flock_north, @flock_west, @flock_south, @move_stop};
%behaviors = {@rendezvous, @flocking, @flock_east, @flock_north, @flock_west, @flock_south, @antirendezvous, @move_stop, @line_x, @line_y};
%behaviors = {@rendezvous, @flocking, @move_right, @move_up, @move_left, @move_down, @antirendezvous, @move_stop};
%behaviors = {@rendezvous, @flocking, @move_forward, @turn_clockwise, @turn_counterclockwise, @align_heading};

%% Plan
time_to_plan_start = tic;
behavior_sequence = plan_behaviors(...
    target_center, target_radius, robot_radius, ...
    inflated_obstacles, behaviors, poses(:,:,1), ti, tf, dt, dT, mapsize);
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

        % Plot
        clf;
        axis([-mapsize mapsize -mapsize mapsize]), hold on;
        plot(target_center(1)+target_radius*cos(0:0.01*pi:2*pi),...
             target_center(2)+target_radius*sin(0:0.01*pi:2*pi));
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
  