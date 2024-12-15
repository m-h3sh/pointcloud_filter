

<h3 align="center">POINTCLOUD FILTERING</h3>

<p align="center"> documentation <br></p>

# pc_test.py <a name = "final_cavity"></a>

Filtering the ZED pointcloud by xyz and RGB thresholds to separate lanes and obstacles.

To run - 
Add the python script into an existing ROS2 package and run as a regular executable.

<h4 align="left">Outputs: </h4>

```
/lane_pc : lane pointcloud
/cones_pc : cones or obstacles pointcloud
```

<h4 align="left">Parameters: </h4>

```
self.x_thresh
self.y_thresh
self.z_thresh
```
These are to remove points that are farther away from the bot, we only transform and filter points which lie inside these bounds.

```
self.lane_z_thresh
```
This is to specify that all white points below this threshold will be counted as lanes.

```
self.lane_thresh
self.cone_thresh
```
These are the [R, G, B] thresholds for the lane and cone filtering.

<h4 align="left">Transformation: </h4>
Currently hardcoded the rotation matrix for the transformation between zed_left_camera_frame to base_link. Update the values for the bot.

```
TODO: get the transform between camera frame and base_link using tf_buffer and use that to transform the points (points->transformed_points)
```

# final_nav.yaml <a name = "mi"></a>

Added voxel layers in our regular nav2 parameters to include pointclouds in the costmap.