#ifndef AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018
#define AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018

#include "VertexPointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeXYPrior.hpp"
#include "ndt_map/ndt_cell.h"
//#include "auto_complete_graph/PriorLoaderInterface.hpp"
//#include "VertexLandmarkNDT.hpp"

namespace g2o {

	class VertexNDTCell : public g2o::VertexPointXYACG {
	protected:

		boost::shared_ptr<perception_oru::NDTCell> _cell;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexNDTCell() : g2o::VertexPointXYACG() {}

		boost::shared_ptr<perception_oru::NDTCell>& getCell(){return _cell;}
		const boost::shared_ptr<perception_oru::NDTCell>& getCell() const {return _cell;}

		void setCell(const boost::shared_ptr<perception_oru::NDTCell>& cell){_cell = cell;}

	};
}


#endif