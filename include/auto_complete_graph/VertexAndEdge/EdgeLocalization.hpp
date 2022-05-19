#ifndef AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018
#define AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018

#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeSE2ACG.hpp"

namespace g2o {

class EdgeLocalization : public g2o::EdgeSE2ACG {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AASS::acg::EdgeInterfaceMalcolm interface;
    EdgeLocalization() : g2o::EdgeSE2ACG(){};
};

}  // namespace g2o
#endif
