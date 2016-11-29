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
	inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraph& acg, grid_map::GridMap& gridMap, double resolution);
	inline void fuseGridMap(const grid_map::GridMap& src, grid_map::GridMap& target, const std::string& layer = "combined", const std::string& layer2 = "combined");
	
	
	inline void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose_vec) {

		Eigen::Affine3d map_origin;
		tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
		Eigen::Affine3d new_map_origin = pose_vec*map_origin;
		tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
	}
	
	inline void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine2d &a2d) {

		//Affine 2d to 3d
		//  Eigen::Rotation2D<double> rot = Eigen::Rotation2D<double>::fromRotationMatrix(a2d.rotation());//Eigen::fromRotationMatrix(a2d.translation());
		double angle = atan2(a2d.rotation()(1,0), a2d.rotation()(0,0));//rot.angle();//acosa2d.rotation()(0,1)/a2d.rotation()(0,0);
		Eigen::Affine3d pose_effi = Eigen::Translation3d(a2d.translation()(0), a2d.translation()(1), 0.) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
		
// 		Eigen::Affine3d pose_effi;
// 		pose_effi.matrix() << a2d(0, 0), a2d(0, 1), 0, a2d(0, 2),
// 					 a2d(1, 0), a2d(1, 1), 0, a2d(1, 2),
// 					 a2d(2, 0), a2d(2, 1), 0, a2d(2, 2),
// 					         0,         0, 0,         0;

		Eigen::Affine3d map_origin;
		tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
		Eigen::Affine3d new_map_origin = pose_effi*map_origin;
		tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
	}
	
	
	
	inline void ACGtoOccupancyGrid(const AASS::acg::AutoCompleteGraph& acg, nav_msgs::OccupancyGrid::Ptr& occ_out){
		//************* TODO : shorten this code !
		//Get max sinze of prior
		grid_map::GridMap map;
		auto edges = acg.getPriorEdges();
		
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
		
		map.setFrameId("/world");
		map.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), 0.1, grid_map::Position(0.0, 0.0));
// 		map.setGeometry(grid_map::Length(1.2, 2.0), 0.4);
		
		map.add("prior"); map.add("ndt"); map.add("all");
		map["prior"].setZero();
		map["ndt"].setZero();
		map["all"].setZero();
  
// 		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		std::cout <<"update the zones again" << std::endl;
		ACGPriortoGridMap(acg, map, 0.1);
		
// 		grid_map::GridMap map_base;
// 		map.setFrameId("/world");
// 		map.setGeometry(grid_map::Length(10, 10), 0.4, grid_map::Position(0.0, 0.0));
		
		nav_msgs::OccupancyGrid* prior_occ = new nav_msgs::OccupancyGrid();
		nav_msgs::OccupancyGrid::ConstPtr ptr_prior_occ(prior_occ);
		grid_map::GridMapRosConverter::toOccupancyGrid(map, "prior", 0, 1., *prior_occ);
		
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		
		
		if(acg.getRobotNodes().size() != 0){
			grids.push_back(ptr_prior_occ);
				
			std::cout <<"update the zones" << std::endl;
			
			for(size_t i = 0 ; i < acg.getRobotNodes().size() ; ++i){
// 				for(size_t i = 0 ; i < 1 ; ++i){
				
//Grid map test
				std::cout << "Node" << std::endl;
				nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();			
// 					initOccupancyGrid(*omap_tmp, 250, 250, 0.4, "/world");
				lslgeneric::toOccupancyGrid(acg.getRobotNodes()[i].getMap(), *omap_tmp, 0.1, "/world");
// 					auto pose = acg.getRobotNodes()[i].getPose();
				auto node = acg.getRobotNodes()[i].getNode();
				auto vertex = node->estimate().toIsometry();
// 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
// 					std::cout << "Move : " << node.matrix() << std::endl;
// 					if(i == 2) exit(0);
				
				std::cout << "Move" << std::endl;
				moveOccupancyMap(*omap_tmp, vertex);
				omap_tmp->header.frame_id = "/world";
				omap_tmp->header.stamp = ros::Time::now();

				nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
				grids.push_back(ptr);
				
			}
			
		}

		std::cout << "Building the final thingy " << grids.size() << std::endl;
		if(grids.size() > 0){
			occ_out = occupancy_grid_utils::combineGrids(grids);
// 				std::cout << "Ref frame " << omap.header.frame_id << std::endl;
			occ_out->header.frame_id = "/world";
			occ_out->header.stamp = ros::Time::now();
			
		}
		std::cout << "Out" << std::endl;
		
		grid_map::GridMap gridMap({"all"});
		grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_out, "all", gridMap);
		cv::Mat originalImageP;
		grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "all", CV_16UC1, 0.0, 1, originalImageP);
		cv::imwrite("/home/malcolm/tmp_all.png", originalImageP);
		
		
		
		

	}
	
	
	
	
	
	inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraph& acg, grid_map::GridMap& gridMap, double resolution){
		auto edges = acg.getPriorEdges();
		
		gridMap.add("prior");
		gridMap["prior"].setZero();
		
		auto it = edges.begin();
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
				gridMap.at("prior", *iterator) = 1;
	// 			publish();
	// 			ros::Duration duration(0.02);
	// 			duration.sleep();
			}
			
		}

// 		return gridMap;
		
	}
	
	
	//TODO
	inline void ACGToGridMap(const AASS::acg::AutoCompleteGraph& acg, grid_map::GridMap& map){
		
		double resol = 0.1;
		
		//************* TODO : shorten this code !
		//Get max sinze of prior
		
		auto edges = acg.getPriorEdges();
		
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
		
		map.setFrameId("/world");
		map.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resol, grid_map::Position(0.0, 0.0));
// 		map.setGeometry(grid_map::Length(1.2, 2.0), 0.4);
		
		map.add("prior"); map.add("ndt"); map.add("all");
		map["prior"].setZero();
		map["ndt"].setZero();
		map["all"].setZero();
  
// 		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		std::cout <<"update the zones" << std::endl;
// 		
		ACGPriortoGridMap(acg, map, resol);
		
		grid_map::Matrix& data3 = map["prior"];
		for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
			const grid_map::Index index(*iterator);
			if(std::isnan(data3(index(0), index(1)))){
				throw std::runtime_error("FUCK 1 already");
			}
		}
		
		
		grid_map::GridMap gridMaptmp({"all"});
		gridMaptmp["all"].setZero();
		gridMaptmp.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resol, grid_map::Position(0.0, 0.0));
// 				
		nav_msgs::OccupancyGrid* prior_occ = new nav_msgs::OccupancyGrid();
		nav_msgs::OccupancyGrid::ConstPtr ptr_prior_occ(prior_occ);
		
		grid_map::GridMapRosConverter::toOccupancyGrid(gridMaptmp, "all", 0 ,1, *prior_occ);
		
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		grids.push_back(ptr_prior_occ);
		
		if(acg.getRobotNodes().size() != 0){
				
			std::cout <<"update the zones" << acg.getRobotNodes().size() << std::endl;
			
			for(size_t i = 0 ; i < acg.getRobotNodes().size() ; ++i){
// 				for(size_t i = 0 ; i < 1 ; ++i){
				
//Grid map test
				std::cout << "Node" << std::endl;
				nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();			
// 					initOccupancyGrid(*omap_tmp, 250, 250, 0.4, "/world");
				lslgeneric::toOccupancyGrid(acg.getRobotNodes()[i].getMap(), *omap_tmp, resol, "/world");
// 					auto pose = acg.getRobotNodes()[i].getPose();
				auto node = acg.getRobotNodes()[i].getNode();
				auto vertex = node->estimate().toIsometry();
// 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
// 					std::cout << "Move : " << node.matrix() << std::endl;
// 					if(i == 2) exit(0);
				
				std::cout << "Move" << std::endl;
				moveOccupancyMap(*omap_tmp, vertex);
				omap_tmp->header.frame_id = "/world";
				omap_tmp->header.stamp = ros::Time::now();

				nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
				grids.push_back(ptr);
				
			}
			
		}

		nav_msgs::OccupancyGrid::Ptr occ_out;
		std::cout << "Building the final thingy " << grids.size() << std::endl;
		if(grids.size() > 0){
			std::cout << "Combine " << grids.size() << std::endl;
			occ_out = occupancy_grid_utils::combineGrids(grids);
// 				std::cout << "Ref frame " << omap.header.frame_id << std::endl;
			occ_out->header.frame_id = "/world";
			occ_out->header.stamp = ros::Time::now();
			
		}
		std::cout << "Out" << std::endl;
		
		grid_map::Matrix& data4 = map["prior"];
		for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
			const grid_map::Index index(*iterator);
			if(std::isnan(data4(index(0), index(1)))){
				throw std::runtime_error("FUCK before");
// 				data2(index(0), index(1)) = -1;
			}
		}
		
		grid_map::GridMap mapNDT;
		//THis ruin prior because they are of different sizes ! Need my custom fuse function :)
		grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_out, "ndt", mapNDT);
		
		grid_map::Matrix& data = mapNDT["ndt"];
		for (grid_map::GridMapIterator iterator(mapNDT); !iterator.isPastEnd(); ++iterator) {
			const grid_map::Index index(*iterator);
			if(std::isnan(data(index(0), index(1)))){
				data(index(0), index(1)) = -1;
			}
		}
		
		grid_map::Matrix& data2 = map["prior"];
		for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
			const grid_map::Index index(*iterator);
			if(std::isnan(data2(index(0), index(1)))){
				throw std::runtime_error("FUCK");
// 				data2(index(0), index(1)) = -1;
			}
		}
// 		
// 		std::cout << map["prior"] << std::endl;
		
		map["all"] = map["prior"];
		
// 		auto diff = map.getPosition() - mapNDT.getPosition();
		
		//Ugliest hack ever ! HACK
		//HACK
// 		map.move(mapNDT.getPosition());
		
// 		assert(map.getPosition()(0) == mapNDT.getPosition()(0) && map.getPosition()(1) == mapNDT.getPosition()(1));
		
// 		grid_map::Index
// 		grid_map::GridMap mapNDT_tmp;
// 		mapNDT_tmp.setGeometry(grid_map::Length(map["prior"].getSize()(0), map["prior"].getSize()(1)), mapNDT.getResolution(), grid_map::Position(mapNDT.getPosition()(0), mapNDT.getPosition()(1)));
		
// 		map["all"] = map["prior"].cwiseMax(map["ndt"]);
		fuseGridMap(mapNDT, map, "ndt", "all");
		
		//HACK ???
// 		mapNDT.move(map.getPosition());
		

	
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
		if(map.getSize()(0) == 0  && map.getSize()(1) == 0){
			std::cerr << "Init the grid map" << std::endl;
			map.setGeometry(grid_map::Length(size_x_cell_count, size_y_cell_count), resolution, grid_map::Position(orig_x, orig_y));
		}
		for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
			grid_map::Position position;
			map.getPosition(*it, position);
			
			
			pcl::PointXYZ pt(position(0), position(1), 0);
			lslgeneric::NDTCell *cell;
			if(!ndt_map->getCellAtPoint(pt, cell)){
// 				occ_grid.data.push_back(-1);
				map.at(layer_name, *it) = 0;
			}
			else if(cell == NULL){
// 				occ_grid.data.push_back(-1);
				map.at(layer_name, *it) = 0;
			}
			else{
				Eigen::Vector3d vec (pt.x,pt.y,pt.z);
				vec = vec-cell->getMean();                  
				double likelihood = vec.dot(cell-> getInverseCov()*vec);
				char s_likelihood;
				if(cell->getOccupancy()!=0.0){
					if(cell->getOccupancy()>0.0){
// 						if(std::isnan(likelihood)) s_likelihood = -1;
						if(std::isnan(likelihood)) s_likelihood = 0;
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
					map.at(layer_name, *it) = 0;
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
	
	/**
	 * @brief fuse src in target
	 * For now only the best value of of both map is kept in the final map.
	 * TODO : The grid map need their centers to be at the exact same physical spot. As trump would say : _BAD_
	 */
	inline void fuseGridMap(const grid_map::GridMap& src, grid_map::GridMap& target, const std::string& layer, const std::string& layer2){
		
		std::cout << "FUSING " << std::endl;
		assert(target.exists(layer2) == true);
		assert(src.exists(layer) == true);
		
// 		std::cout << "Fusing " << std::endl;
		grid_map::GridMap modifiedMap;
		grid_map::GridMapCvProcessing::changeResolution(src, modifiedMap, target.getResolution());
		
		if(target.getSize()(0) == modifiedMap.getSize()(0) && target.getSize()(1) == modifiedMap.getSize()(1)){
			target[layer2] = src[layer] + target[layer2];
		}
		else{
			grid_map::Matrix& data_targ = target[layer2];
			grid_map::Matrix& data_src = modifiedMap[layer];
			auto t_size = target.getSize();
			auto s_size = modifiedMap.getSize();
			
			auto pos_targ = target.getPosition();
			auto pos_src = src.getPosition();
// 			assert(pos_targ(0) == pos_src(0) && pos_targ(1) == pos_src(1));
			
			int max_x, max_y;
			max_x = std::max(t_size(0), s_size(0));
			max_y = std::max(t_size(1), s_size(1));
			
			Eigen::MatrixXf out(max_x, max_y);
			out.setZero();
			
			int t1 = t_size(0);
			int t2 = t_size(1);
			int s1 = s_size(0);
			int s2 = s_size(1);
			
			std::cout << "targ" << (max_x - t_size(0))/2 << " "<< (max_y - t_size(1))/2 << " " << t1 << " " << s1 <<std::endl;
			
			out.block( (max_x - t_size(0))/2, (max_y - t_size(1))/2, t1, t2) = data_targ;
			
			std::cout << "src" << (max_x - s_size(0))/2 << " "<< (max_y - s_size(1))/2 << " " << t2 << " " << s2 <<std::endl;
			
			out.block( (max_x - s_size(0))/2, (max_y - s_size(1))/2, s1, s2) = out.block( (max_x - s_size(0))/2, (max_y - s_size(1))/2, s1, s2).cwiseMax(data_src);
			
			std::cout << "assigning target" << std::endl;
			target[layer2].resize(max_x, max_y);
			target[layer2] = out;
			
		}
		
		
		
// 		grid_map::Matrix& data = target[layer2];
// 		
// 		for (grid_map::GridMapIterator it(target); !it.isPastEnd(); ++it) {
// 			grid_map::Position position;
// 			target.getPosition(*it, position);
// 			try{
// 				double val = modifiedMap.atPosition(layer, position);
// // 				std::cout << "val norm " << val << " " << target.at(layer2, *it) << std::endl;
// 				
// 				if(target.at(layer2, *it) < val || std::isnan(target.at(layer2, *it)) ){
// // 					std::cout << "Tru" << std::endl;
// 					target.at(layer2, *it) = val;
// 				}
// // 				std::cout << "val norm " << val << " " << target.at(layer2, *it) << std::endl;
// 			}
// 			catch (const std::out_of_range& e) {
// 				std::cout << "Out of Range but it should be fine. \n";
// 			}
// 			
// 		}
		
	}
	
	inline void fuseGridMap(std::vector<grid_map::GridMap>& maps, std::vector<std::string>& layers,  grid_map::GridMap& combinedGrid, const std::string& frame_id, double resolution){
// 		grid_map::GridMap combinedGrid({"combined"});
		
		double size_x = -1, size_y = -1;
		for(auto it = maps.begin(); it != maps.end() ; ++it){
// 			std::cout << "Size other " << it->getLength() << std::endl;
			if(size_x < it->getLength()(0)) size_x = it->getLength()(0);
			if(size_y < it->getLength()(1)) size_y = it->getLength()(1);
		}
		
		combinedGrid.setFrameId(frame_id);
		combinedGrid.setGeometry(grid_map::Length(size_x, size_y), resolution);
		combinedGrid.add("combined");
		grid_map::Matrix& data_targ = combinedGrid["combined"];
		data_targ.setZero();
		
// 		std::cout << "Sizes" << size_x << " " << size_y << std::endl;
		
		int i = 0;
		for(auto it = maps.begin(); it != maps.end() ; ++it){
			fuseGridMap(*it, combinedGrid, layers[i]);
			++i;
// 			cv::Mat originalImageP2fu;
// 			grid_map::GridMapCvConverter::toImage<unsigned short, 1>(combinedGrid, "combined", CV_16UC1, 0.0, 1, originalImageP2fu);
// 			cv::imwrite("/home/malcolm/grid2fusedtemp.png", originalImageP2fu);
// 			exit(0);
		}
		assert(combinedGrid.getSize()(0) == combinedGrid["combined"].rows());
		assert(combinedGrid.getSize()(1) == combinedGrid["combined"].cols());
	}
	
	
	
	
	
}
}


#endif