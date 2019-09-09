function ptcloud = read_bin(bin_path, height)

%% defualt arg
if nargin < 2
    height = 1.9;
end
    
%% Read 
fid = fopen(bin_path, 'rb'); raw_data = fread(fid, [4 inf], 'single'); fclose(fid);

points = raw_data(1:3, :)';
points(:, 3) = points(:, 3) + height; % z in car coord.
ptcloud = pointCloud(points);

intensities = raw_data(4, :)';
ptcloud.Intensity = intensities;

end % end of function
