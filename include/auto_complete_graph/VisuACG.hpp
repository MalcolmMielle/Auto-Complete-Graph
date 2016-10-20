#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016

#include "auto_complete_graph/ACG.hpp"
#include "occupancy_grid_utils/combine_grids.h"


namespace AASS {

namespace acg{	
	
	
	class VisuAutoCompleteGraph{ 
	
	protected:
		ros::NodeHandle _nh;
		ros::Publisher _last_ndtmap;
		
	public:
		VisuAutoCompleteGraph(){
			_last_ndtmap = _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg", 10);
		}
		void toRviz(const AutoCompleteGraph& acg);
		
	private:
		
		bool fuseNDTMap(const AutoCompleteGraph& acg, nav_msgs::OccupancyGridPtr& final);
		bool printNDTMap(lslgeneric::NDTMap* map, const std::__cxx11::string& frame_name, ndt_map::NDTMapMsg& mapmsg);
		void printPrior(const std::vector<g2o::VertexSE2Prior*>& prior_corners);
		void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Vector3d &pose_vec);
		
		//TO TEST
		Eigen::Affine3d vector3dToAffine3d(const Eigen::Vector3d& vec){
			Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));
			return t;
		}
		
	};
	

	inline bool AASS::acg::VisuAutoCompleteGraph::printNDTMap(lslgeneric::NDTMap* map, const std::string& frame_name, ndt_map::NDTMapMsg& mapmsg)
	{
		return lslgeneric::toMessage(map, mapmsg, frame_name);
	}


	inline void AASS::acg::VisuAutoCompleteGraph::printPrior(const std::vector< g2o::VertexSE2Prior* >& prior_corners)
	{

	}

	inline void AASS::acg::VisuAutoCompleteGraph::toRviz(const AASS::acg::AutoCompleteGraph& acg)
	{
		nav_msgs::OccupancyGrid::Ptr final;
		fuseNDTMap(acg, final);
		final->header.frame_id = "/world";
		final->header.stamp = ros::Time::now();
		std::cout << "Ref frame " << final->header.frame_id << std::endl; 
		_last_ndtmap.publish<nav_msgs::OccupancyGrid>(*final);
		
		
	}

	inline void AASS::acg::VisuAutoCompleteGraph::moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Vector3d &pose_vec) {

		auto pose = vector3dToAffine3d(pose_vec);
		Eigen::Affine3d map_origin;
		tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
		Eigen::Affine3d new_map_origin = pose*map_origin;
		tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
	}

	inline bool AASS::acg::VisuAutoCompleteGraph::fuseNDTMap(const AASS::acg::AutoCompleteGraph& acg, nav_msgs::OccupancyGrid::Ptr& final)
	{
		
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		
		auto nodeMap = acg.getRobotNodes();
		auto it = nodeMap.begin();
		int i = 0 ;
		for(it ; it != nodeMap.end() ; ++it){
			
			auto map = it->getMap();
			auto node = it->getNode();
			auto pos = node->estimate();
						
			std::cout << "First : " << i << std::endl;
			++i;
			nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();
			std::cout << "Bad map ? " << std::endl;
			std::cout << "To occupancy grid. net map with cell num : " << map->numberOfActiveCells() << std::endl;
			lslgeneric::toOccupancyGrid(map, *omap_tmp, 1.0, "/world");
			std::cout << "To move grid" << std::endl;
			moveOccupancyMap(*omap_tmp, pos.toVector());
			nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
			grids.push_back(ptr);
			
		}
		
			std::cout << "To combine grid" << std::endl;
		final = occupancy_grid_utils::combineGrids(grids);
		
		
	}

	
	
	
}
}

#endif