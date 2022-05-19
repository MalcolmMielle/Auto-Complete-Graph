#ifndef AUTOCOMPLETEGRAPH_EDGESE2PRIOR_10112017
#define AUTOCOMPLETEGRAPH_EDGESE2PRIOR_10112017

#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeSE2ACG.hpp"

namespace g2o {

class VertexSE2Prior;
}

namespace g2o {

class EdgeSE2Prior_malcolm : public g2o::EdgeSE2ACG {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AASS::acg::EdgeInterfaceMalcolm interface;
    EdgeSE2Prior_malcolm() : g2o::EdgeSE2ACG() {}
    Eigen::Vector2d getOrientation2D(const g2o::VertexSE2Prior& from) const;
};

}  // namespace g2o
#endif