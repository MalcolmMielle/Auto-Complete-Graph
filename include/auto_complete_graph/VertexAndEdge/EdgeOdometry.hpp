#ifndef AUTOCOMPLETEGRAPH_EDGEODOMETRY_10112017
#define AUTOCOMPLETEGRAPH_EDGEODOMETRY_10112017

#include "EdgeSE2ACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace g2o
{

	class EdgeOdometry_malcolm : public g2o::EdgeSE2ACG
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		AASS::acg::EdgeInterfaceMalcolm interface;
		EdgeOdometry_malcolm() : g2o::EdgeSE2ACG(){};
	};

}

#endif