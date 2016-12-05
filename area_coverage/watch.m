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
trial_id = 'trial_20161205T045935';
data_file = 'N_10_tf_00100d0000_dt_00000d1000_dT_00010d0000';

%% Load data
folder = [data_folder,'/',trial_id];
data = load([folder,'/',data_file]);

%% Make video
video = VideoWriter([folder,'/',trial_id,'.avi']);
open(video);
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
for ts=1:size(poses,3)
    t = (ts-1)*dt;
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
    xlabel('X (m)'), ylabel('Y (m)');
    title(['Time = ',num2str(t)]);
    %legend('Target Location',...
    %       'Location','EastOutside');
    %drawnow;
    frame = getframe;
    writeVideo(video,frame);
end
close(video);
