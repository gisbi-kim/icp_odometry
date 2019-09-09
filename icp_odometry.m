function [motion] = icp_odometry(cur_ptcloud, prev_ptcloud, VOXEL_SIZE, mode, MAX_ITER)
% prev: fixed  
% curr: moving

%% mode 
if(nargin < 4)
   mode = 'plane'; 
   MAX_ITER = 30;
end

%% down for fast 
cur_ptcloud_down = downsample(cur_ptcloud, VOXEL_SIZE);
prev_ptcloud_down = downsample(prev_ptcloud, VOXEL_SIZE);

%% registration 
if(strcmp(mode, 'plane'))
    tform = pcregrigid(cur_ptcloud_down, prev_ptcloud_down, 'Metric', 'pointToPlane', 'MaxIterations', MAX_ITER);
else
    tform = pcregrigid(cur_ptcloud_down, prev_ptcloud_down, 'Metric', 'pointToPoint', 'MaxIterations', MAX_ITER);
end

motion = transpose(tform.T);

end

