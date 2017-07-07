# Auto Complete Graph.

## Paper

The auto complete graph method is presented in [this article](https://www.arxiv.org/abs/1702.05087).

## How to use

### Load the prior

The class `PriorLoaderInterface` is used to load the map. See `BasementFull` as an example.

### Optimize

Use `acg_node_review` to run, replace BasementFull inside by your class to load the prior.

In Rviz, one can give an approximation of the position of the prior compared to robot map using the Publish point button. Only two links are needed to initialise.

The optimization is then done automatically,

## Dependencies
