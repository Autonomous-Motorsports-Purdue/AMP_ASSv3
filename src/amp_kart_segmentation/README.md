# RANSAC Segmentation

Executable: `segmentation`

During obstacle detection, we use data from the poinhtcloud, which contains both ground and non-ground features. Since we don't want the kart to see the ground as an obstacle, we have to segment the ground plane from the pointcloud.

One issue is that since the kart moves through a significant range of 6DOF poses, simply segmenting based on a z-axis threshold is not enough to call it a day. Instead, we have to use RANSAC, and algorithm which matches a virtual ground plane to its most likely position on the sensed pointcloud. However, the ground has a slight curvature that actually makes it fit poorly to a flat plane.

This code segments the pointcloud into approximately "square" sectors and runs RANSAC independently on each section. In addition, since RANSAC is sensitive to point density, we also pass it through a VoxelFilter to impose a maximum density.

## Files

`launch/segmentation.launch.py`: Example launch file for `segmentation`, with params set from yaml file and topic remappings
`params/segmentation.params.yaml`: Good default parameters for `segmentation`
`rviz/default.rviz`: Sample rviz config for displaying input and output. Run with `rviz2 -d /path/to/default.rviz`
`src/segmentation.cpp` The node and implementation
`CmakeLists.txt`: Finds and links PCL. If this fails, install `libpcl-dev` to your package manager.

## Parameters (and default values):

`threshold`: double - Allowed deviation from the ground plane that is still considered ground

`negative`: bool - True publishes non-ground parts and False publishes ground parts

`voxel_size`: double - Voxelization (gridifying) minimum distance between points before they are removed

The following are dimensions of the crop box

`minx`: double
`maxx`: double
`miny`: double
`maxy`: double
`minz`: double
`maxz`: double

`ring_gap`: double - Ring_gap approximately controls the radial span and arc length of each sector

## Topics

`~/input`: `sensor_msgs/Pointcloud2` default: `/velodyne_points`

`~/output`: `sensor_msgs/Pointcloud2` default: `/nonground`
