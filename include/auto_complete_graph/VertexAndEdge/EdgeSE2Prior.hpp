#ifndef AUTOCOMPLETEGRAPH_EDGESE2PRIOR_10112017
#define AUTOCOMPLETEGRAPH_EDGESE2PRIOR_10112017

#include "g2o/types/slam2d/edge_se2.h"
#include "EdgeInterfaceMalcolm.hpp"

namespace  g2o {

	class VertexSE2Prior;
}


namespace AASS {
namespace acg{

	
class EdgeSE2Prior_malcolm : public g2o::EdgeSE2
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeInterfaceMalcolm interface;
	EdgeSE2Prior_malcolm() : g2o::EdgeSE2(){}
	Eigen::Vector2d getOrientation2D(const g2o::VertexSE2Prior& from) const;

};

}
}
#endif