Header header #standard header information

grid_map_msgs/GridMap[] ndt_maps_om #All robot submap
geometry_msgs/Transform[] robot_poses #transformation between the previous submap and the next.
grid_map_msgs/GridMap prior #Prior map as a gridMap with resolution 0.1 and frame "/world"