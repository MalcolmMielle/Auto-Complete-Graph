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
		ros::Publisher _last_ndtmap2;
		nav_msgs::OccupancyGrid omap;
		int _nb_of_zone;
		AutoCompleteGraph* _acg;
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		
	public:
		VisuAutoCompleteGraph(AutoCompleteGraph* acg) : _nb_of_zone(0){
			_last_ndtmap = _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg", 10);
			_last_ndtmap2 = _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg2", 10);
			_acg = acg;
// 			initOccupancyGrid(omap, 500, 500, 0.4, "/world");
		}
// 		void toRviz(const AutoCompleteGraph& acg);
		
		void updateRviz(){
			if(_nb_of_zone != _acg->getRobotNodes().size()){
				
				std::cout <<"update the zones" << std::endl;
				
				for(size_t i = _nb_of_zone ; i < _acg->getRobotNodes().size() ; ++i){
// 				for(size_t i = 0 ; i < 1 ; ++i){
					
					nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();			
// 					initOccupancyGrid(*omap_tmp, 250, 250, 0.4, "/world");
					lslgeneric::toOccupancyGrid(_acg->getRobotNodes()[i].getMap(), *omap_tmp, 0.4, "/world");
					auto pose = _acg->getRobotNodes()[i].getPose();
// 					auto vertex = node->estimate().toVector();
// 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
					std::cout << "Move : " << pose.matrix() << std::endl;
// 					if(i == 2) exit(0);
					moveOccupancyMap(*omap_tmp, pose);
					omap_tmp->header.frame_id = "/world";
					omap_tmp->header.stamp = ros::Time::now();
					//TODO
// 					fuseOcc(omap_tmp, omap);
// 					if(i == 0){
// 						_last_ndtmap.publish<nav_msgs::OccupancyGrid>(omap_tmp);
// 					}
// 					if(i == 2 ){
// 						_last_ndtmap2.publish<nav_msgs::OccupancyGrid>(omap_tmp);
// 					}
					nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
					grids.push_back(ptr);
				}
				
			}
			_nb_of_zone = _acg->getRobotNodes().size();
// 			omap.header.frame_id = "/world";
// 			omap.header.stamp = ros::Time::now();
			nav_msgs::OccupancyGrid::Ptr final;
			if(grids.size() > 0){
				final = occupancy_grid_utils::combineGrids(grids);
// 				std::cout << "Ref frame " << omap.header.frame_id << std::endl;
				final->header.frame_id = "/world";
				final->header.stamp = ros::Time::now();
				_last_ndtmap.publish<nav_msgs::OccupancyGrid>(*final);
			}
			
			
			
		}
		
	private:
		
// 		bool fuseNDTMap(const AutoCompleteGraph& acg, nav_msgs::OccupancyGridPtr& final);
// 		bool printNDTMap(lslgeneric::NDTMap* map, const std::string& frame_name, ndt_map::NDTMapMsg& mapmsg);
// 		void printPrior(const std::vector<g2o::VertexSE2Prior*>& prior_corners);
		void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose_vec);
		
		//TO TEST
		Eigen::Affine3d vector3dToAffine3d(const Eigen::Vector3d& vec){
			Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));
			return t;
		}
		
// 		bool initOccupancyGrid(nav_msgs::OccupancyGrid& occ_grid, int width, int height, double res, const std::string& frame_id);
// 		bool toOccupancyGrid(lslgeneric::NDTMap *ndt_map, nav_msgs::OccupancyGrid &occ_grid, double resolution,std::string frame_id);
		
// 		void fuseOcc(nav_msgs::OccupancyGrid& source, nav_msgs::OccupancyGrid& dest);
		
	};
	

// 	inline bool AASS::acg::VisuAutoCompleteGraph::printNDTMap(lslgeneric::NDTMap* map, const std::string& frame_name, ndt_map::NDTMapMsg& mapmsg)
// 	{
// 		return lslgeneric::toMessage(map, mapmsg, frame_name);
// 	}
// 
// 
// 	inline void AASS::acg::VisuAutoCompleteGraph::printPrior(const std::vector< g2o::VertexSE2Prior* >& prior_corners)
// 	{
// 
// 	}
// 
// 	inline void AASS::acg::VisuAutoCompleteGraph::toRviz(const AASS::acg::AutoCompleteGraph& acg)
// 	{
// 		nav_msgs::OccupancyGrid::Ptr final;
// 		fuseNDTMap(acg, final);
// 		final->header.frame_id = "/world";
// 		final->header.stamp = ros::Time::now();
// 		std::cout << "Ref frame " << final->header.frame_id << std::endl; 
// 		_last_ndtmap.publish<nav_msgs::OccupancyGrid>(*final);
// 		
// 		
// 	}

// 	inline void AASS::acg::VisuAutoCompleteGraph::moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Vector3d &pose_vec) {
// 
// 		auto pose = vector3dToAffine3d(pose_vec);
// 		Eigen::Affine3d map_origin;
// 		tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
// 		Eigen::Affine3d new_map_origin = pose*map_origin;
// 		tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
// 	}
// 	
	inline void AASS::acg::VisuAutoCompleteGraph::moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose_vec) {

		Eigen::Affine3d map_origin;
		tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
		Eigen::Affine3d new_map_origin = pose_vec*map_origin;
		tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
	}

// 	inline bool AASS::acg::VisuAutoCompleteGraph::fuseNDTMap(const AASS::acg::AutoCompleteGraph& acg, nav_msgs::OccupancyGrid::Ptr& final)
// 	{
// 		
// 		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
// 		
// 		auto nodeMap = acg.getRobotNodes();
// 		auto it = nodeMap.begin();
// 		int i = 0 ;
// 		for(it ; it != nodeMap.end() ; ++it){
// 			
// 			auto map = it->getMap();
// 			auto node = it->getNode();
// 			auto pos = node->estimate();
// 						
// 			std::cout << "First : " << i << std::endl;
// 			++i;
// 			nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();
// 			std::cout << "Bad map ? " << std::endl;
// 			std::cout << "To occupancy grid. net map with cell num : " << map->numberOfActiveCells() << std::endl;
// 			lslgeneric::toOccupancyGrid(map, *omap_tmp, 1.0, "/world");
// 			std::cout << "To move grid" << std::endl;
// // 			moveOccupancyMap(*omap_tmp, pos.toVector());
// 			nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
// 			grids.push_back(ptr);
// 			
// 		}
// 		
// 			std::cout << "To combine grid" << std::endl;
// 		final = occupancy_grid_utils::combineGrids(grids);
// 		
// 		
// 	}
// 
// 	
// 	inline bool VisuAutoCompleteGraph::initOccupancyGrid(nav_msgs::OccupancyGrid& occ_grid, int width, int height, double res, const std::string& frame_id)
// 	{
// 		occ_grid.info.width=width;
// 		occ_grid.info.height=height;
// 		occ_grid.info.resolution=res;
// 		occ_grid.info.map_load_time=ros::Time::now();
// 		occ_grid.info.origin.position.x=-50;
// 		occ_grid.info.origin.position.y=-50;
// 		occ_grid.header.stamp=ros::Time::now();
// 		occ_grid.header.frame_id=frame_id;
// 		
// 		for(int iy = 0; iy < width; iy++) {
// 			for(int ix = 0; ix < height; ix++) {
// 				occ_grid.data.push_back(-1);
// 			}
// 		}
// 	}

	
/*	
	inline bool AASS::acg::VisuAutoCompleteGraph::toOccupancyGrid(lslgeneric::NDTMap *ndt_map, nav_msgs::OccupancyGrid &occ_grid, double resolution,std::string frame_id)
	{
		//works only for 2D case
    double size_x, size_y, size_z;
    int size_x_cell_count, size_y_cell_count;
    double cen_x, cen_y, cen_z;
    double orig_x, orig_y;
    ndt_map->getGridSizeInMeters(size_x,size_y,size_z);
    ndt_map->getCentroid(cen_x,cen_y,cen_z);
    orig_x=cen_x-size_x/2.0;
    orig_y=cen_y-size_y/2.0;
	size_x_cell_count=int(size_x/resolution);
    size_y_cell_count=int(size_y/resolution);
	
	std::cout << "orig " << orig_x << " " << orig_y << " " << cen_x << " " << cen_y << " " << size_x << " " << size_y << " " << size_x_cell_count <<  " "<< size_y_cell_count << " " << resolution << std::endl;
	std::cout << "orig " << occ_grid.info.origin.position.x << " " << occ_grid.info.origin.position.x << " " << cen_x << " " << cen_y << " " << size_x << " " << size_y << " " << occ_grid.info.width <<  " "<< occ_grid.info.height << " " << occ_grid.info.resolution << std::endl;
	
// 	exit(0);
    
//     occ_grid.info.width=size_x_cell_count;
//     occ_grid.info.height=size_y_cell_count;
//     occ_grid.info.resolution=resolution;
//     occ_grid.info.map_load_time=ros::Time::now();
//     occ_grid.info.origin.position.x=orig_x;
//     occ_grid.info.origin.position.y=orig_y;
//     occ_grid.header.stamp=ros::Time::now();
//     occ_grid.header.frame_id=frame_id;
// //     for(double py=orig_y+resolution/2.0;py<orig_y+size_x;py+=resolution){
//     //   for(double px=orig_x+resolution/2.0;px<orig_x+size_x;px+=resolution){
    for(int iy = 0; iy < occ_grid.info.width; iy++) {
      for(int ix = 0; ix < occ_grid.info.height; ix++) {
		  
		//Find the point to study 
		double px = occ_grid.info.origin.position.x + occ_grid.info.resolution*ix + occ_grid.info.resolution*0.5;
        double py = occ_grid.info.origin.position.y + occ_grid.info.resolution*iy + occ_grid.info.resolution*0.5;

        pcl::PointXYZ pt(px,py,0);
        lslgeneric::NDTCell *cell;
        if(!ndt_map->getCellAtPoint(pt, cell)){
			//NO cell == do nothing
//           occ_grid.data.push_back(-1);
        }
        else if(cell == NULL){
			//NO cell == do nothing
//           occ_grid.data.push_back(-1);
        }
        else{
          Eigen::Vector3d vec (pt.x,pt.y,pt.z);
          vec = vec-cell->getMean();                  
          double likelihood = vec.dot(cell-> getInverseCov()*vec);
          char s_likelihood;
          if(cell->getOccupancy()!=0.0){
			  std::cout << "occ " << cell->getOccupancy() << std::endl;
			if(cell->getOccupancy()>0.0){
				if(std::isnan(likelihood)) s_likelihood = -1;
				likelihood = exp(-likelihood/2.0) + 0.1;
				likelihood = (0.5+0.5*likelihood);
				s_likelihood=char(likelihood*100.0);
				
				std::cout << "likelihood" << likelihood << std::endl;
				
				if(likelihood >1.0) s_likelihood =100;
				
				std::cout << "Good cell" << std::endl;
				occ_grid.data.at( (iy * occ_grid.info.width) + ix) = s_likelihood;
			}
			else{
				std::cout << "Good cell" << std::endl;
				occ_grid.data.at( (iy * occ_grid.info.width) + ix) = 0;
			}
          }
          else{
// 			  std::cout << "occ :( " << cell->getOccupancy() << std::endl;
			  //NO cell == do nothing
//              occ_grid.data.push_back(-1);
          }
        }
      }
    }    
//     exit(0);
    return true;
  } */
  
  
  
	 //BUG
// 	void VisuAutoCompleteGraph::fuseOcc(nav_msgs::OccupancyGrid& source, nav_msgs::OccupancyGrid& dest)
// 	{
// 		
// 		double topleft_x = dest.info.origin.position.x + dest.info.resolution*0.5;
// 		double topleft_y = dest.info.origin.position.y + dest.info.resolution*0.5;
// 		
// // 		double topright_x = dest.info.origin.position.x + dest.info.resolution * 0 + dest.info.resolution*0.5;
// 		double topright_y = dest.info.origin.position.y + dest.info.resolution * dest.info.width + dest.info.resolution*0.5;
// 		
// 		double bottomleft_x = dest.info.origin.position.x + dest.info.resolution * dest.info.height + dest.info.resolution*0.5;
// // 		double bottomleft_y = dest.info.origin.position.y + dest.info.resolution * 0 + dest.info.resolution*0.5;
// 		
// // 		double bottomright_x = dest.info.origin.position.x + dest.info.resolution* dest.info.height + dest.info.resolution*0.5;
// // 		double bottomright_y = dest.info.origin.position.y + dest.info.resolution* dest.info.width + dest.info.resolution*0.5;
// 		
// // 		assert(source.info.width == dest.info.width);
// // 		assert(source.info.height == dest.info.height);
// 		for(int iy = 0; iy < source.info.width; iy++) {
// 			for(int ix = 0; ix < source.info.height; ix++) {
// 				
// 				double px = source.info.origin.position.x + source.info.resolution*ix + source.info.resolution*0.5;
// 				double py = source.info.origin.position.y + source.info.resolution*iy + source.info.resolution*0.5;
// 
// 				//Check if the point is in the dest image or not.
// // 				if(px >= topleft_x && px <= bottomleft_x){
// // 					if(py >= topleft_y && py <= topright_y){
// 						if(source.data.at( (iy * source.info.width) + ix) >= 0){
// 							
// 							int place_x = (px - dest.info.resolution*0.5 - dest.info.origin.position.x) / dest.info.resolution;
// 							int place_y = (py - dest.info.resolution*0.5 - dest.info.origin.position.x) / dest.info.resolution;
// 							
// 							dest.data.at( (place_y * dest.info.width) + place_x) = source.data.at( (iy * source.info.width) + ix);
// 						}
// // 					}
// // 				}
// // 				else{
// // 					throw std::runtime_error("OUTSIDE OF THE MAP!");
// // 				}
// 			}
// 		}
// 
// 	}

	
	
}
}

#endif