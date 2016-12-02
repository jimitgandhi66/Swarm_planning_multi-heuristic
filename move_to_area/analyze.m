%% Swarm Simulation Video
% Description : Switching Behaviors - Video
% Author      : Sasanka Nagavalli 
% Date        : February 7, 2016
% Other Files :

%% Clean up
clear;
clc;
clf;
close all;

%% Paths
addpath('../behaviors');
data_folder = '../data';

%% Find data
trial_id = 'trial_20161120T162251';
data_file = 'N_10_tf_00050d0000_dt_00000d1000_dT_00005d0000';

%% Load data
folder = [data_folder,'/',trial_id];
data = load([folder,'/',data_file]);

%% Make image
N = data.N;
O = data.O;
mapsize = data.mapsize;
ti = data.ti;
tf = data.tf;
dt = data.dt;
target_center = data.target_center;
target_radius = data.target_radius;
robot_radius = data.robot_radius;
labels = data.labels;
poses = data.poses;
obstacles = data.obstacles;

%ts = round((t-ti)/dt) + 1;
clf;
axis([-mapsize mapsize -mapsize mapsize]), hold on;
plot(target_center(1)+target_radius*cos(0:0.01*pi:2*pi),...
     target_center(2)+target_radius*sin(0:0.01*pi:2*pi));
for i=1:O
    obs = obstacles{i};
    fill(obs(:,1),obs(:,2),'g');
end
for i=1:N
    p1 = poses(i,1,:);
    p2 = poses(i,2,:);
    plot(p1(:),p2(:));
end
xlabel('X (m)'), ylabel('Y (m)');
%title(['Time = ',num2str(t)]);
%legend('Target Location',...
%       'Location','EastOutside');
%drawnow;

print('-dpng',...
    [folder,'/',trial_id,'_trajectory']);
