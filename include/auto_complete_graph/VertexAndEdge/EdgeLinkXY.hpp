#ifndef AUTOCOMPLETEGRAPH_EDGELINKXY_10112017
#define AUTOCOMPLETEGRAPH_EDGELINKXY_10112017

#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeSE2PointXYACG.hpp"

namespace g2o {

class EdgeLinkXY_malcolm : public g2o::EdgeSE2PointXYACG {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AASS::acg::EdgeInterfaceMalcolm interface;
    EdgeLinkXY_malcolm() : EdgeSE2PointXYACG(){};
};

}  // namespace g2o
#endif