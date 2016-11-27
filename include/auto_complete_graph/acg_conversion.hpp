#ifndef AUTOCOMPLETEGRAPH_ACG_CONVERSION_25112016
#define AUTOCOMPLETEGRAPH_ACG_CONVERSION_25112016

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "grid_map_ros/GridMapRosConverter.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include "occupancy_grid_utils/combine_grids.h"

#include "ACG.hpp"

namespace AASS{
namespace acg{

	inline bool toGridMap(lslgeneric::NDTMap* ndt_map, grid_map::GridMap& map, double resolution, std::__cxx11::string frame_id, std::string layer_name);
	
	inline void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose_vec) {

		Eigen::Affine3d map_origin;
		tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
		Eigen::Affine3d new_map_origin = pose_vec*map_origin;
		tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
	}
	
	
	inline grid_map::GridMap ACGPriortoGridMap(const AASS::acg::AutoCompleteGraph& acg, double resolution){
		auto edges = acg.getPriorEdges();
		
		
		//************* TODO : shorten this code !
		//Get max sinze of prior
		double max_x, min_x, max_y, min_y;
		bool flag_init = false;
		auto it = edges.begin();
		for(it ; it != edges.end() ; ++it){
			for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
				geometry_msgs::Point p;
				g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
				auto vertex = ptr->estimate().toVector();
				if(flag_init == false){
					flag_init = true;
					max_x = vertex(0);
					max_y = vertex(1);
					min_x = vertex(0);
					min_y = vertex(1);
				}
				else{
					if(max_x < vertex(0)){
						max_x = vertex(0);
					}
					if(max_y < vertex(1)){
						max_y = vertex(1);
					}
					if(min_x > vertex(0)){
						min_x = vertex(0);
					}
					if(min_y > vertex(1)){
						min_y = vertex(1);
					}
				}
				
			}
			
		}
		
		max_x = std::abs(max_x);
		max_y = std::abs(max_y);
		min_x = std::abs(min_x);
		min_y = std::abs(min_y);
		
		double size_x, size_y;
		if(max_x > min_x){
			size_x = max_x + 10;
		}
		else{
			size_x = min_x + 10;
		}
		if(max_y > min_y){
			size_y = max_y + 10;
		}
		else{
			size_y = min_y + 10;
		}

		/***********************************/
		
		
		grid_map::GridMap gridMap({"prior"});
		gridMap.setGeometry(grid_map::Length(size_x, size_y), resolution, grid_map::Position(0.0, 0.0));
		gridMap.setFrameId("map");
		
		it = edges.begin();
		for(it ; it != edges.end() ; ++it){
			
			
			std::vector<Eigen::Vector2d> points;
			
			for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
				geometry_msgs::Point p;
				g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
				auto vertex = ptr->estimate().toVector();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				Eigen::Vector2d veve; veve << vertex(0), vertex(1);
				points.push_back(veve);
			}
			
			assert(points.size() == 2);
			
			for (grid_map::LineIterator iterator(gridMap, points[0], points[1]);
				!iterator.isPastEnd(); ++iterator) {
				gridMap.at("prior", *iterator) = 1.0;
	// 			publish();
	// 			ros::Duration duration(0.02);
	// 			duration.sleep();
			}
			
		}

		return gridMap;
		
	}
	
	
	//TODO
	inline grid_map::GridMap ACGToGridMap(const AASS::acg::AutoCompleteGraph& acg){
		
		grid_map::GridMap map({"elevation"});
		map.setFrameId("/world");
// 		map.setGeometry(grid_map::Length(1.2, 2.0), 0.4);
  
  
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		std::cout <<"update the zones" << std::endl;
		
		for(size_t i = 0 ; i < acg.getRobotNodes().size() ; ++i){
			
			toGridMap(acg.getRobotNodes()[i].getMap(), map, 0.4, "/world", "elevation");
			
			
			
// 				for(size_t i = 0 ; i < 1 ; ++i){
			
// 			nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();			
// 					initOccupancyGrid(*omap_tmp, 250, 250, 0.4, "/world");
// 			lslgeneric::toOccupancyGrid(acg->getRobotNodes()[i].getMap(), *omap_tmp, 0.4, "/world");
// // 					auto pose = _acg->getRobotNodes()[i].getPose();
// 			auto node = acg->getRobotNodes()[i].getNode();
// 			auto vertex = node->estimate().toIsometry();
// // 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
// // 					std::cout << "Move : " << node.matrix() << std::endl;
// // 					if(i == 2) exit(0);
// 			
// 			moveOccupancyMap(*omap_tmp, vertex);
// 			
// 			omap_tmp->header.frame_id = "/world";
// 			omap_tmp->header.stamp = ros::Time::now();
// 			//TODO
// // 					fuseOcc(omap_tmp, omap);
// // 					if(i == 0){
// // 						_last_ndtmap.publish<nav_msgs::OccupancyGrid>(omap_tmp);
// // 					}
// // 					if(i == 2 ){
// // 						_last_ndtmap2.publish<nav_msgs::OccupancyGrid>(omap_tmp);
// // 					}
// 			nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
// 			grids.push_back(ptr);
			
			
// 			geometry_msgs::Point p;
// 			auto vertex2 = node->estimate().toVector();
// 			//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
// 			p.x = vertex2(0);
// 			p.y = vertex2(1);
// 			p.z = 0;
			
			
		}
// 		
// 	
// 		
// 	// 			omap.header.frame_id = "/world";
// 	// 			omap.header.stamp = ros::Time::now();
// 		nav_msgs::OccupancyGrid::Ptr final;
// 		if(grids.size() > 0){
// 			final = occupancy_grid_utils::combineGrids(grids);
// 	// 				std::cout << "Ref frame " << omap.header.frame_id << std::endl;
// 			final->header.frame_id = "/world";
// 			final->header.stamp = ros::Time::now();
// 			
// 		}
// 		
// 		grid_map::GridMapRosConverter converter;
// 		converter.fromOccupancyGrid(*final, "elevation", map);
// 		
// 		return map;
	
	}
	
	
	
	
	void toOccupancyGrid(){
		
	}
	
	
	
	/**
	*
	* \brief builds ocuupancy grid message
	* \details Builds 2D occupancy grid map based on 2D NDTMap
	* @param[in] ndt_map 2D ndt map to conversion
	* @param[out] occ_grid 2D cost map
	* @param[in] resolution desired resolution of occupancy map
	* @param[in] name of cooridnation frame for the map (same as the NDT map has)
	* 
	*/
	inline bool toGridMap(lslgeneric::NDTMap *ndt_map, grid_map::GridMap &map, double resolution,std::string frame_id, std::string layer_name){//works only for 2D case
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
		
// 		occ_grid.info.width=size_x_cell_count;
// 		occ_grid.info.height=size_y_cell_count;
// 		occ_grid.info.resolution=resolution;
// 		occ_grid.info.map_load_time=ros::Time::now();
// 		occ_grid.info.origin.position.x=orig_x;
// 		occ_grid.info.origin.position.y=orig_y;
// 		occ_grid.header.stamp=ros::Time::now();
// 		occ_grid.header.frame_id=frame_id;
		
		map.setFrameId(frame_id);
		map.setGeometry(grid_map::Length(size_x_cell_count, size_y_cell_count), resolution, grid_map::Position(orig_x, orig_y));
		
		for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
			grid_map::Position position;
			map.getPosition(*it, position);
			
			
			pcl::PointXYZ pt(position(0), position(1), 0);
			lslgeneric::NDTCell *cell;
			if(!ndt_map->getCellAtPoint(pt, cell)){
// 				occ_grid.data.push_back(-1);
				map.at(layer_name, *it) = -1;
			}
			else if(cell == NULL){
// 				occ_grid.data.push_back(-1);
				map.at(layer_name, *it) = -1;
			}
			else{
				Eigen::Vector3d vec (pt.x,pt.y,pt.z);
				vec = vec-cell->getMean();                  
				double likelihood = vec.dot(cell-> getInverseCov()*vec);
				char s_likelihood;
				if(cell->getOccupancy()!=0.0){
					if(cell->getOccupancy()>0.0){
						if(std::isnan(likelihood)) s_likelihood = -1;
						likelihood = exp(-likelihood/2.0) + 0.1;
						likelihood = (0.5+0.5*likelihood);
						s_likelihood=char(likelihood*100.0);
						if(likelihood >1.0) s_likelihood =100;
// 						occ_grid.data.push_back(s_likelihood);
						map.at(layer_name, *it) = s_likelihood;
					}
					else{
// 						occ_grid.data.push_back(0);
						map.at(layer_name, *it) = 0;
					}
				}
				else{
// 					occ_grid.data.push_back(-1);
					map.at(layer_name, *it) = -1;
				}
			}
			
		}
		
		return true;
// 		// for(double py=orig_y+resolution/2.0;py<orig_y+size_x;py+=resolution){
// 		//   for(double px=orig_x+resolution/2.0;px<orig_x+size_x;px+=resolution){
// 		for(int iy = 0; iy < size_y_cell_count; iy++) {
// 			for(int ix = 0; ix < size_x_cell_count; ix++) {
// 				double px = orig_x + resolution*ix + resolution*0.5;
// 				double py = orig_y + resolution*iy + resolution*0.5;
// 
// 				pcl::PointXYZ pt(px,py,0);
// 				lslgeneric::NDTCell *cell;
// 				if(!ndt_map->getCellAtPoint(pt, cell)){
// 					occ_grid.data.push_back(-1);
// 				}
// 				else if(cell == NULL){
// 					occ_grid.data.push_back(-1);
// 				}
// 				else{
// 					Eigen::Vector3d vec (pt.x,pt.y,pt.z);
// 					vec = vec-cell->getMean();                  
// 					double likelihood = vec.dot(cell-> getInverseCov()*vec);
// 					char s_likelihood;
// 					if(cell->getOccupancy()!=0.0){
// 						if(cell->getOccupancy()>0.0){
// 							if(std::isnan(likelihood)) s_likelihood = -1;
// 							likelihood = exp(-likelihood/2.0) + 0.1;
// 							likelihood = (0.5+0.5*likelihood);
// 							s_likelihood=char(likelihood*100.0);
// 							if(likelihood >1.0) s_likelihood =100;
// 							occ_grid.data.push_back(s_likelihood);
// 						}
// 						else{
// 							occ_grid.data.push_back(0);
// 						}
// 					}
// 					else{
// 						occ_grid.data.push_back(-1);
// 					}
// 				}
// 			}
// 		}    
// 		return true;
	} 
	
}
}


#endif