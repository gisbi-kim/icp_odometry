function [ptcloud_down] = downsample(ptcloud, voxel_size)
ptcloud_down = pcdownsample(ptcloud, 'gridAverage', voxel_size);
end

