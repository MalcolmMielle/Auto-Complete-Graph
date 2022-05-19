#ifndef AUTOCOMPLETEGRAPH_EDGENDTCELLOBSERVATION_03072018
#define AUTOCOMPLETEGRAPH_EDGENDTCELLOBSERVATION_03072018

#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeSE2PointXYACG.hpp"

namespace g2o {

class EdgeNDTCellObservation : public g2o::EdgeSE2PointXYACG {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeNDTCellObservation() : EdgeSE2PointXYACG(){};
};

}  // namespace g2o
#endif