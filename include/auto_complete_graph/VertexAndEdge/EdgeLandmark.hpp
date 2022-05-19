#ifndef AUTOCOMPLETEGRAPH_EDGELANDMARK_10112017
#define AUTOCOMPLETEGRAPH_EDGELANDMARK_10112017

#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeSE2PointXYACG.hpp"

namespace g2o {

class EdgeLandmark_malcolm : public g2o::EdgeSE2PointXYACG {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AASS::acg::EdgeInterfaceMalcolm interface;

    Eigen::Vector2d original_observation = Eigen::Vector2d::Zero();
    EdgeLandmark_malcolm() : EdgeSE2PointXYACG(){};
};

}  // namespace g2o
#endif