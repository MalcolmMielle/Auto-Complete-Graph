#ifndef AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018
#define AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018

#include "VertexPointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeXYPrior.hpp"
//#include "auto_complete_graph/PriorLoaderInterface.hpp"
//#include "VertexLandmarkNDT.hpp"

namespace g2o {

	class VertexNDTCell : public g2o::VertexPointXYACG {
	protected:

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexNDTCell() : g2o::VertexPointXYACG() {}

	};
}


#endif