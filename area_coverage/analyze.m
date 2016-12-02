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
trial_id = 'trial_20161120T165625';
data_file = 'N_10_tf_00100d0000_dt_00000d1000_dT_00010d0000';

%% Load data
folder = [data_folder,'/',trial_id];
data = load([folder,'/',data_file]);

%% Make image
N = data.N;
O = data.O;
map = data.map;
ti = data.ti;
tf = data.tf;
dt = data.dt;
robot_radius = data.robot_radius;
labels = data.labels;
poses = data.poses;
map_grid = data.map_grid;
obstacles = data.obstacles;

clf;
axis([0 map.size_x 0 map.size_y]), hold on;
image([0 map.size_x],[0 map.size_y],repmat(map_grid(:,:,end),1,1,3));
colormap('gray');
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
