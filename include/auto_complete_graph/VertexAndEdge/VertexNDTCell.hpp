#ifndef AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018
#define AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018

#include "VertexPointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeXYPrior.hpp"
//#include "VertexSE2RobotLocalization.hpp"
#include "ndt_map/ndt_cell.h"
//#include "auto_complete_graph/PriorLoaderInterface.hpp"
//#include "VertexLandmarkNDT.hpp"

namespace g2o {

	class VertexNDTCell : public g2o::VertexPointXYACG {
	protected:

		boost::shared_ptr<perception_oru::NDTCell> _cell;
		std::set<VertexNDTCell*> _equivalent_cells;
//		std::set<VertexSE2RobotLocalization*> _localizations;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexNDTCell() : g2o::VertexPointXYACG() {}

		boost::shared_ptr<perception_oru::NDTCell>& getCell(){return _cell;}
		const boost::shared_ptr<perception_oru::NDTCell>& getCell() const {return _cell;}
		std::set<VertexNDTCell*>& getEquivalentNDTCells(){return _equivalent_cells;}
		const std::set<VertexNDTCell*>& getEquivalentNDTCells() const {return _equivalent_cells;}
//		const std::set<VertexSE2RobotLocalization*>& getLocalizations() const {return _localizations;}
//		std::set<VertexSE2RobotLocalization*>& getLocalizations(){return _localizations;}

//		void addLocalization(VertexSE2RobotLocalization* loc){_localizations.insert(loc);}
		void setCell(const boost::shared_ptr<perception_oru::NDTCell>& cell){_cell = cell;}

	};
}


#endif