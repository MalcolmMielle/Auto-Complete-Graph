# Auto Complete Graph (ACG).

Using emergency maps to help robots save you in emergencies

## Description

The goal of this program is described [in this blog post](https://malcolmmielle.wordpress.com/2017/08/07/using-emergency-maps-to-help-robots-save-you-in-emergencies/) and can be summarized by this sentence:

> a method to integrate an emergency map into a robot map, so that the robot can plan its way toward places it has not yet explored.

![Result example](https://raw.githubusercontent.com/MalcolmMielle/Auto-Complete-Graph/SSRR2017/Images/result.png)

## Paper

The auto-complete graph uses a circular strategy to integrate an emergency map and a robot build map in a global representation. The robot build a map of the environment using NDT mapping, and in parallel do localization in the emergency map using Monte-Carlo Localization. Corners are extracted in both the robot map and the emergency map. Using the information from the localization, a graph-SLAM is created where observation of the emergency map corner are determined using the localization covariance and the position of the emergency map's corners compared to the position of corners detected in the robot map. The graph is further constrained by having emergency map walls able to stretch or shrink but being hard to rotate. This is done because emergency maps have usually local scaling problems but do have correct topology.

A first version of the auto complete graph method is presented in [this article](https://www.arxiv.org/abs/1702.05087) on arxiv and was publish in [SSRR2017]() where it got the best student paper award. This version integrates the emergency map in the robot map using a specific graph matching strategy.


## To run the code

Two launch file are used to run the code: one for the mapping and localization, and another for the graph-SLAM optimization.

### Mapping and localization parameters:

* use\_mcl: if true, use MCL in the emergency map.
* use\_graph\_map\_registration: if true, use NDT mapping.
* zfilter\_min: 
* fraction:
* cutoff:
* init\_var:
* scale\_gaussian\_mcl: scaling factor for the gaussian in MCL localization
* cell\_neighborhood\_size\_mcl\_in\_meters: if mcl is allozed to use cell further away from the actual cell scan, the parameter is the size of the neighbor that MCL is allowed to search.
* use\_euclidean\_mcl: use the euclidean distance as a measure for the cell fitness in MCL.
* use\_mean\_score\_mcl: use the mean value of all cell within the neighbor
* use\_euclidean\_for\_long\_distances: only use the euclidean distance if the cell is further than the distance\_euclid parameter.
* use\_hybrid\_strategy\_mcl: use an hybrid strategy in MCL where either we find a cell corresponding exactly to the scan and we use it as is, or we consider the neighbor and use the mean of all cell fitness. This parameter forces the use of the euclidean distance.
* cov\_x\_mcl: starting cov along the x axis for the MCL.
* cov\_y\_mcl: starting cov along the y axis for the MCL.
* cov\_yaw\_mcl: starting cov along the yaw axis for the MCL.


### ACG parameters

* pause\_for\_testing: if true, the program will stop at every step.
* export\_iteration\_count: if true, export the number of iteration per kernel in the file "/home/malcolm/ros_catkin_ws/lunar_ws/iterations.dat". The file needs to exist.
* add\_noise\_odometry: if true, noise is added to the odometry edges.
* not\_incremental\_optimization: if true, and if add\_noise\_odometry is true, the graph will use the position of the previous node in the graph_map instead of the previous node in acg when adding new node. This is useful if, when adding noise, the graph is not updated at each turn. Indeed, since the localization is based on the position in the graph map but the position in the ACG has noise, the prior matching will be wrong otherwise.
* use\_prior: if true, integrate the emergency map.
* optimize\_prior: if true, the emergency map is optimized to fit the robot map.
* use\_robot\_maps: if true, the mapping given by the robot is used.
* use\_corner: extract corners from the robot map.
* use\_corner\_orientation: use the corner orientation as a parameter for corner matching.
* corner\_covariance: use an approximation of the corner covariance given the NDT used to find it.
* own\_registration: register the submaps.
* mcl\_observation\_on\_prior: use MCL to find correspondences between the emergency map and the robot map. UNUSED
* links\_prior\_classic\_ssrr: use old strategy. DOESN'T WORK ANYMORE.
* use\_mcl\_cov\_to\_find\_prior\_observed: UNUSED
* world\_frame: the world frame
* sensor\_frame: the sensor frame
* covariance\_to\_find\_links: UNUSED
* gaussian\_scaling\_factor: scaling factor of the MCL covariance.
* threshold\_score\_link\_creation: probability above which the corner are considered the same and matched. This should be only slightly above zero.
* prior\_file: file localization for the emergency map image.
* max\_deviation\_corner\_in\_prior: minimum angle to extract a corner from the emergency map.
* scale: scale of the emergency map.

<remap from="acg_node_localization/prior_ndt" to ="/ndt_map_init_mcl"/>

## Run the code example

Run both launch files `hannover.launch` and `acg.launch`

* The parameters for ACG are loaded from a file. The file is determined by a string in `acg_node_review.cpp` named `parameters_for_ACG`. An example of parameter file is present in `ACG_params/param.txt`

* In Rviz, one can give an approximation of the position\_in\_robot\_frame of the prior compared to robot map using the Publish point button. Only two links are needed to initialize. First click on corner in the emergency map and then in the equivalent corner in the robot map.

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

The results can be visualized using the [auto-complete-graph visualization]() package.

## Details about the method

### Corner matching using MCL

The MCL poses can be considered as _"where the robot thinks he is in the emergency map when considering the laser scanner input"_. Hence, here is the strategy used to find association between the emergency map and the robot map:

* Every MCL pose is associated with an equivalent robot map pose and a submap.
* For every corner in the submap, its coordinate are changed to assume it is seen from the MCL position instead of the robot map pose, i.e. we translate the submap from the robot map pose to the MCL pose.
* We score the likeliness that a corner correspond to a emergency map corner by using the MCL covariance:
	* We calculate the mahalanobis distance using the MCL covariance: `(pose_prior - pose_landmark).dot( mcl_cov_inverse * (pose_prior - pose_landmark));`
	* We then calculate the probability of the corners being the same: `prob = exp(mahalanobis distance)`
	This probability will be only slightly superior to zero if the corner pose _"fit in the covariance"_. Hence we only need to look for a score slightly superior to zero to get a matching ; we use 5%.
* If the corner match we create an observation from the robot pose to emergency map corner. This observation's covariance is the equivalent MCL pose covariance.

