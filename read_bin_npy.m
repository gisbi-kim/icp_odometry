function ptcloud = read_bin_npy(bin_path)

xyzi = readNPY(bin_path);

xyz = xyzi(:, 1:3);
intensities = xyzi(:, 4);

ptcloud = pointCloud(xyz);
ptcloud.Intensity = intensities;

end % end of function
