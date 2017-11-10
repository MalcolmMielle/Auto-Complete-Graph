#ifndef AUTOCOMPLETEGRAPH_EDGELINKXY_10112017
#define AUTOCOMPLETEGRAPH_EDGELINKXY_10112017

#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "EdgeInterfaceMalcolm.hpp"

namespace AASS {
namespace acg{	
	
class EdgeLinkXY_malcolm : public g2o::EdgeSE2PointXY
{
public:
	EdgeInterfaceMalcolm interface;
	//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeLinkXY_malcolm() : EdgeSE2PointXY() {};

};

}
}
#endif