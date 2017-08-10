# Auto Complete Graph (ACG).

## Paper

The auto complete graph method is presented in [this article](https://www.arxiv.org/abs/1702.05087).

## Run the code example

This should all be soon ros parameters but for now here is how you can test the code:

* Use `roslaunch ndt_feature gustav_radar_tf.launch` to create the ndt_graph that is going to be processed by the ACG. Modify the file so that it reads your bag file. Use this [bag file](http://aass.oru.se/Research/mro/data/tutorials/mapping.bag) if your only testing the algorithm.

* The parameters for ACG are loaded from a file. The file is determined by a string in `acg_node_review.cpp` named `parameters_for_ACG`. An example of parameter file is present in `ACG_params/param.txt`

* Use `rosrun auto_complete_graph acg_node_review` to run the algorithm.

* In Rviz, one can give an approximation of the position of the prior compared to robot map using the Publish point button. Only two links are needed to initialise.

### How to use your own prior

The class `PriorLoaderInterface.hpp` is used to load prior image and detected corners. Just inherite from this class to create your own prior loader. See an example in the class `basementFull.hpp`. This only argument needed for PriorLoaderInterface is the name of the file image where the prior is.

## Dependencies

* [BetterGraph](https://github.com/MalcolmMielle/BetterGraph)
* [VoDiGrEx](https://github.com/MalcolmMielle/VoDiGrEx)
* [G2O](https://github.com/RainerKuemmerle/g2o)
* [perception_oru](https://github.com/OrebroUniversity/perception_oru)
* [ndt_feature](https://github.com/MalcolmMielle/ndt_feature_graph)
* OpenCV
* Eigen
* Boost

