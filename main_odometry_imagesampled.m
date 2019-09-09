clear; clc;
addpath(genpath('../npy-matlab-master'));

%% config 
config_imagesampled

%% dataset 
scanfile_names = listdir(scan_dir);
scanfile_paths = fullfile(scan_dir, scanfile_names);

num_scans = length(scanfile_names);

% test_scan = fullfile(scanfile_paths{1});

%% gt pose 
gt_poses_raw = readNPY('../submap_nodes.npy');
gt_poses = gt_poses_raw(:, 3:end);


%% odometry only 
se3poses = cell(1, num_scans);
xyzposes = zeros(num_scans, 3);
xyzposes_gt = zeros(num_scans, 3);

prev_ptcloud = NaN;
prev_pose = NaN;

start_idx = 1;
end_idx = num_scans;
% end_idx = 100;
for scan_idx = start_idx:end_idx
    
    % get current data
    img_xyz = (double(imread(scanfile_paths{scan_idx}))-10000)/100;
    img_xyz = reshape(img_xyz, [size(img_xyz,1) * size(img_xyz, 2), 3]);
    cur_ptcloud = pointCloud(img_xyz);
    
    % initial pose is origin 
    if(scan_idx < 2)
        cur_pose = eye(4); 
        se3poses{scan_idx} = cur_pose;
        xyzposes(scan_idx, :) = take_xyz(cur_pose);
        xyzposes_gt(scan_idx, :) = take_xyz(cur_pose);
        
        prev_ptcloud = cur_ptcloud;
        prev_pose = cur_pose;
        continue; 
    end
    
    % calc relative motion 
    tic
    motion = icp_odometry(cur_ptcloud, prev_ptcloud, VOXEL_SIZE, 'plane', 30); % including downsample for fast converges
    cur_pose = prev_pose * motion; % multiply at the right because matlab se3 is transposed compare to conventional case
    toc
    
    % save 
    se3poses{scan_idx} = cur_pose;
    xyzposes(scan_idx, :) = take_xyz(cur_pose);
    xyzposes_gt(scan_idx, :) = gt_poses(scan_idx, [4,8,12])- gt_poses(1, [4,8,12]);    
    
    % update prev variables 
    prev_ptcloud = cur_ptcloud;
    prev_pose = cur_pose;
    
    % status log 
    if(rem(scan_idx, 100) == 0)
       disp(scan_idx); 
    end
    
    if(rem(scan_idx, 20) == 0)
       
        num_viz_points = scan_idx - 1;

        figure(2); clf;
        point_colors = jet(num_viz_points);

        xyzposes_gt_xyz = [xyzposes_gt(:, 2), -1 * xyzposes_gt(:, 1), xyzposes_gt(:, 3)];
        pcshow(xyzposes(1:num_viz_points, :), 'r', 'MarkerSize', 100); hold on;
        pcshow(xyzposes_gt_xyz(1:num_viz_points, :), 'g', 'MarkerSize', 100); hold on;

        view(0, 90);
        xlabel('x (m)'); ylabel('y (m)'); % in camera coord
        axis equal;

    end
end


%% trajectory viewer 
num_viz_points = scan_idx - 1;

figure(2); clf;
point_colors = jet(num_viz_points);

xyzposes_gt_xyz = [xyzposes_gt(:, 2), -1 * xyzposes_gt(:, 1), xyzposes_gt(:, 3)];
pcshow(xyzposes(1:num_viz_points, :), 'r', 'MarkerSize', 100); hold on;
pcshow(xyzposes_gt_xyz(1:num_viz_points, :), 'g', 'MarkerSize', 100); hold on;

view(0, 90);
xlabel('x (m)'); ylabel('y (m)'); % in camera coord
axis equal;



