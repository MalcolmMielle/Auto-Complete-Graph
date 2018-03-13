#ifndef AUTOCOMPLETEGRAPH_EDGELINKXY_10112017
#define AUTOCOMPLETEGRAPH_EDGELINKXY_10112017

#include "EdgeSE2PointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace g2o{
	
class EdgeLinkXY_malcolm : public g2o::EdgeSE2PointXYACG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	AASS::acg::EdgeInterfaceMalcolm interface;
	//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeLinkXY_malcolm() : EdgeSE2PointXYACG() {};

};

}
#endif