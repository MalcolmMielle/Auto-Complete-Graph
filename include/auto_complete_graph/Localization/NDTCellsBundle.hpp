#ifndef AUTOCOMPLETEGRAPH_NDTCELLBUNDLE_14092018
#define AUTOCOMPLETEGRAPH_NDTCELLBUNDLE_14092018


#include "ndt_map/ndt_cell.h"
#include "auto_complete_graph/VertexAndEdge/VertexNDTCell.hpp"
#include "auto_complete_graph/VertexAndEdge/VertexSE2RobotLocalization.hpp"

namespace AASS {

	namespace acg {


		class NDTCellsBundle {

		protected:

			std::set<g2o::VertexNDTCell*> _cells;
			std::set<g2o::VertexSE2RobotLocalization*> _robot_poses;

		public:
			NDTCellsBundle(){}

			void addNDTCell(g2o::VertexNDTCell* ndt_cell){_cells.insert(ndt_cell);}
			void addNDTCell(g2o::VertexSE2RobotLocalization* robot_pose){_robot_poses.insert(robot_pose);}

			std::set<g2o::VertexNDTCell*> getCells(){ return _cells;}
			const std::set<g2o::VertexNDTCell*> getCells() const { return _cells;}
			std::set<g2o::VertexSE2RobotLocalization*> getRobotPOses(){return _robot_poses;}
			const std::set<g2o::VertexSE2RobotLocalization*> getRobotPOses() const {return _robot_poses;}
		};
	}
}


#endif