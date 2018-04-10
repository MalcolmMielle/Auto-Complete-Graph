# Auto Complete Graph (ACG).


Using emergency maps to help robots save you in emergencies

## Description

The goal of this program is described [in this blog post](https://malcolmmielle.wordpress.com/2017/08/07/using-emergency-maps-to-help-robots-save-you-in-emergencies/) and can be summarized by this sentence:

> a method to integrate an emergency map into a robot map, so that the robot can plan its way toward places it has not yet explored.

![Result example](https://raw.githubusercontent.com/MalcolmMielle/Auto-Complete-Graph/SSRR2017/Images/result.png)

## Paper

The auto complete graph method is presented in [this article](https://www.arxiv.org/abs/1702.05087) on arxiv.


## Run the code example

This should all be soon ros parameters but for now here is how you can test the code:

* Use `roslaunch ndt_feature gustav_radar_tf.launch` to create the ndt_graph that is going to be processed by the ACG. Modify the file so that it reads your bag file. Use this [bag file](http://aass.oru.se/Research/mro/data/tutorials/mapping.bag) if your only testing the algorithm.

* The parameters for ACG are loaded from a file. The file is determined by a string in `acg_node_review.cpp` named `parameters_for_ACG`. An example of parameter file is present in `ACG_params/param.txt`

* Use `rosrun auto_complete_graph acg_node_review` to run the algorithm.

* In Rviz, one can give an approximation of the position_in_robot_frame of the prior compared to robot map using the Publish point button. Only two links are needed to initialize.

### How to use your own prior

The class `PriorLoaderInterface.hpp` is used to load prior image and detected corners. Just inherit from this class to create your own prior loader. See an example in the class `basementFull.hpp`. This only argument needed for PriorLoaderInterface is the name of the file image where the prior is.

## Dependencies

* [BetterGraph](https://github.com/MalcolmMielle/BetterGraph)
* [VoDiGrEx](https://github.com/MalcolmMielle/VoDiGrEx)
* [G2O](https://github.com/RainerKuemmerle/g2o)
* [perception_oru](https://github.com/OrebroUniversity/perception_oru)
* [ndt_feature](https://github.com/MalcolmMielle/ndt_feature_graph)
* [occupancy_grid_utils](https://github.com/clearpathrobotics/occupancy_grid_utils)
* [grid_map](https://github.com/ethz-asl/grid_map)
* OpenCV
* Eigen
* Boost

## ROS 

### Topics

The results of the ACG can be fetched on the topic `/auto_complete_graph_rviz_small_optimi/acg_maps`. The message format is as follow:

```
Header header #standard header information

ndt_map/NDTVectorMapMsg ndt_maps #All robot submap and associated translations
grid_map_msgs/GridMap prior #Prior map as a gridMap with resolution 0.1 and frame "/world"
```
Each ndt map is centered where the corresponding robot pose node is situated. The robot pose node can be found be adding all transformations.

Another topic where one can find the results is `/auto_complete_graph_rviz_small_optimi/acg_maps_om`. The message format is as follow:

```
Header header #standard header information

grid_map_msgs/GridMap[] ndt_maps_om #All robot submap as grid maps. The layer is name 'ndt'
geometry_msgs/Pose[] robot_poses #transformation between the previous submap and the next.
grid_map_msgs/GridMap prior #Prior map as a gridMap with resolution 0.1 and origin frame "/world"
```
Each ndt map is centered where the corresponding robot pose node is situated.

