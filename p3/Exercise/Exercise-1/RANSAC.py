# Import PCL module
import pcl

LEAF_SIZE = 0.01

# Load Point Cloud file
data = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter
vox = data.make_voxel_grid_filter()
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
data_ds = vox.filter()

# PassThrough filter
axis_min = 0.6
axis_max = 1.1
filter_axis = 'z'
passthrough = data_ds.make_passthrough_filter()
passthrough.set_filter_field_name(filter_axis)
passthrough.set_filter_limits(axis_min, axis_max)
data_pt = passthrough.filter()

# RANSAC plane segmentation
seg = data_pt.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
max_distance = 0.01
seg.set_distance_threshold(max_distance)
inliers, coefficients = seg.segment()


# Extract inliers
extracted_inliers = data_pt.extract(inliers, negative=False)

# Save pcd for table
# pcl.save(cloud, filename)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)

# Extract outliers
extracted_outliers = data_pt.extract(inliers, negative=True)

# Save pcd for tabletop objects
filename = 'tabletop_filtered.pcd'
pcl.save(extracted_outliers, filename)

