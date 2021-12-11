#ifndef AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018
#define AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018

#include "EdgeSE2ACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace g2o
{

	class EdgeLocalization : public g2o::EdgeSE2ACG
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		AASS::acg::EdgeInterfaceMalcolm interface;
		EdgeLocalization() : g2o::EdgeSE2ACG(){};
	};

}
#endif
