#ifndef AUTOCOMPLETEGRAPH_ACG_CONVERSION_25112016
#define AUTOCOMPLETEGRAPH_ACG_CONVERSION_25112016

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "grid_map_ros/GridMapRosConverter.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include "occupancy_grid_utils/combine_grids.h"
#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_map/NDTVectorMapMsg.h"
#include "auto_complete_graph/ACGMaps.h"
#include "auto_complete_graph/ACGMapsOM.h"

#include "ACGBase.hpp"

#include "auto_complete_graph/Localization/ACG_localization.hpp"

namespace AASS{
namespace acg{

	inline bool toGridMap(perception_oru::NDTMap* ndt_map, grid_map::GridMap& map, double resolution, std::__cxx11::string frame_id, std::string layer_name);

	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, grid_map::GridMap& gridMap, double resolution);

	template<>
	inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorSE2, g2o::VertexSE2RobotPose, g2o::EdgeSE2Prior_malcolm>& acg, grid_map::GridMap& gridMap, double resolution);

	template<>
	inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>& acg, grid_map::GridMap& gridMap, double resolution);

	inline void fuseGridMap(const grid_map::GridMap& src, grid_map::GridMap& target, grid_map::GridMap& out_grid, const std::string& layer = "combined", const std::string& layer2 = "combined", const std::string& final_layer = "combined");

	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGNdtNodetoVecGrids(
		const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, std::vector< g2o::VertexSE2RobotPose* >::const_iterator& it_start, std::vector< g2o::VertexSE2RobotPose* >::const_iterator& it_end,
		std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids_out
	);
	
	
	
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


	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline nav_msgs::OccupancyGrid::Ptr ACGNDTtoOcc(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, nav_msgs::OccupancyGrid::ConstPtr& ptr_prior_occ, double resol){
		
		
		
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
        if(acg.getRobotNodes().size() != 0){
            grids.push_back(ptr_prior_occ);
				
            std::cout <<"update the zones" << acg.getRobotNodes().size() << std::endl;
			
            for(size_t i = 0 ; i < acg.getRobotNodes().size() ; ++i){
// 				for(size_t i = 0 ; i < 1 ; ++i){
				
//Grid map test
                std::cout << "Node" << std::endl;
                nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();
// 					initOccupancyGrid(*omap, 250, 250, 0.4, "/world");
                perception_oru::toOccupancyGrid(acg.getRobotNodes()[i]->getMap().get(), *omap, resol, "/world");
// 					auto pose = acg.getRobotNodes()[i].getPose();
                auto node = acg.getRobotNodes()[i];
                auto vertex = node->estimate().toIsometry();
// 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
// 					std::cout << "Move : " << node.matrix() << std::endl;
// 					if(i == 2) exit(0);
				
                std::cout << "Move" << std::endl;
                moveOccupancyMap(*omap, vertex);
                omap->header.frame_id = "/world";
                omap->header.stamp = ros::Time::now();

                nav_msgs::OccupancyGrid::ConstPtr ptr(omap);
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
		return occ_out;
	}


	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGtoOccupancyGrid(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, nav_msgs::OccupancyGrid::Ptr& occ_out, int start = 0, int end = -1){
		//************* TODO : shorten this code !
		//Get max sinze of prior
		grid_map::GridMap map;
		auto edges = acg.getPrior()->getEdges();
		
		double max_x, min_x, max_y, min_y;
		bool flag_init = false;
		auto it = edges.begin();
		for(it ; it != edges.end() ; ++it){
			for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
				geometry_msgs::Point p;
				g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
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
// 		map["prior"].setZero();
		map["ndt"].setZero();
		map["all"].setZero();
		
		std::cout << "Pos again1" << map.getPosition() << std::endl;
  
// 		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		std::cout <<"update the zones again" << std::endl;
		ACGPriortoGridMap<Prior, VertexPrior, EdgePrior>(acg, map, 0.1);
		
		std::cout << "Pos again" << map.getPosition() << std::endl;
		
		
// 		grid_map::GridMap map_base;
// 		map.setFrameId("/world");
// 		map.setGeometry(grid_map::Length(10, 10), 0.4, grid_map::Position(0.0, 0.0));
		
		nav_msgs::OccupancyGrid* prior_occ = new nav_msgs::OccupancyGrid();
		nav_msgs::OccupancyGrid::ConstPtr ptr_prior_occ(prior_occ);
		grid_map::GridMapRosConverter::toOccupancyGrid(map, "prior", 0, 1., *prior_occ);
		
		std::cout << "POsition " << prior_occ->info.origin.position << std::endl;
		std::cout << "ORientation " << prior_occ->info.origin.orientation << std::endl;
// 		exit(0);
		
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		grids.push_back(ptr_prior_occ);
		
// 		if(acg.getRobotNodes().size() != 0){
// // 			grids.push_back(ptr_prior_occ);
// 				
// 			std::vector<g2o::VertexSE2RobotPose*>::const_iterator it;
// 			std::vector<g2o::VertexSE2RobotPose*>::const_iterator it_end;
// 			std::cout <<"update the zones" << std::endl;
// 			if(start < acg.getRobotNodes().size() && start >= 0){
// 				it = acg.getRobotNodes().begin() + start;
// 			}
// 			else{
// 				it = acg.getRobotNodes().begin();
// 			}
// 			
// 			if(end < start && end != -1){
// 				throw std::runtime_error("End pointer is before the start. Can draw zones backward in ACG_CONVERSION.hpp");
// 			}
// 			
// 			if(end < acg.getRobotNodes().size() && end >= 0 && end >= start){
// 				it_end = acg.getRobotNodes().begin() + end;
// 			}
// 			else{
// 				it_end = acg.getRobotNodes().end();
// 			}
// 			
// 			ACGNdtNodetoVecGrids(acg, it, it_end, grids);
// 			
// // 			for(size_t i = 0 ; i < acg.getRobotNodes().size() ; ++i){
// // 			for(it ; it != acg.getRobotNodes().end() ; ++it){
// // // 				for(size_t i = 0 ; i < 1 ; ++i){
// // 				
// // //Grid map test
// // 				std::cout << "Node" << std::endl;
// // 				nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();			
// // // 					initOccupancyGrid(*omap, 250, 250, 0.4, "/world");
// // 				perception_oru::toOccupancyGrid(it->getMap(), *omap, 0.1, "/world");
// // // 					auto pose = acg.getRobotNodes()[i].getPose();
// // 				auto node = it->getNode();
// // 				auto vertex = node->estimate().toIsometry();
// // // 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
// // // 					std::cout << "Move : " << node.matrix() << std::endl;
// // // 					if(i == 2) exit(0);
// // 				
// // 				std::cout << "Move" << std::endl;
// // 				moveOccupancyMap(*omap, vertex);
// // 				omap->header.frame_id = "/world";
// // 				omap->header.stamp = ros::Time::now();
// // 
// // 				nav_msgs::OccupancyGrid::ConstPtr ptr(omap);
// // 				grids.push_back(ptr);
// // 				
// // 			}
// 			
// 		}

		std::cout << "Building the final thingy " << grids.size() << std::endl;
		if(grids.size() > 0){
			occ_out = occupancy_grid_utils::combineGrids(grids);
// 				std::cout << "Ref frame " << omap.header.frame_id << std::endl;
			occ_out->header.frame_id = "/world";
			occ_out->header.stamp = ros::Time::now();
			
		}
		std::cout << "Out" << std::endl;
		
// 		grid_map::GridMap gridMap({"all"});
// 		grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_out, "all", gridMap);
// 		cv::Mat originalImageP;
// 		grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "all", CV_16UC1, 0.0, 1, originalImageP);
// 		cv::imwrite("/home/malcolm/tmp_all.png", originalImageP);
		
		
		
		

	}


	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGNdtNodetoVecGrids(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, std::vector< g2o::VertexSE2RobotPose* >::const_iterator& it_start, std::vector< g2o::VertexSE2RobotPose* >::const_iterator& it_end, std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids_out){
		
		for(it_start ; it_start != it_end ; ++it_start){
// 				for(size_t i = 0 ; i < 1 ; ++i){
			
//Grid map test
			std::cout << "Node" << std::endl;
			nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();		
			std::cout << "Node size" << (*it_start)->getMap()->getAllInitializedCellsShared().size() << std::endl;
			assert((*it_start)->getMap()->getAllInitializedCellsShared().size() == (*it_start)->getMap().get()->getAllInitializedCellsShared().size());
			
			std::cout << "Node size" << (*it_start)->getMap()->getAllCellsShared().size() << std::endl;
			assert((*it_start)->getMap()->getAllCellsShared().size() == (*it_start)->getMap().get()->getAllCellsShared().size());
// 					initOccupancyGrid(*omap, 250, 250, 0.4, "/world");
			perception_oru::toOccupancyGrid((*it_start)->getMap().get(), *omap, 0.3, "/world");
// 					auto pose = acg.getRobotNodes()[i].getPose();
			auto node = *it_start;
			auto vertex = node->estimate().toIsometry();
// 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
// 					std::cout << "Move : " << node.matrix() << std::endl;
// 					if(i == 2) exit(0);
			
			std::cout << "Move" << std::endl;
			moveOccupancyMap(*omap, vertex);
			omap->header.frame_id = "/world";
			omap->header.stamp = ros::Time::now();

			nav_msgs::OccupancyGrid::ConstPtr ptr(omap);
			grids_out.push_back(ptr);
			
		}
		
		
		
	}


//	inline pcl::PointCloud<pcl::PointXYZ>::Ptr ACGPriortoPointCloud(const AASS::acg::AutoCompleteGraph& acg, double resolution, double varz){
//
//		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
//		int nb_points = 0;
//		auto edges = acg.getPrior()->getEdges();
//		std::cout << "Converting edges : " << edges.size() << std::endl;
//
//		for(auto it = edges.begin(); it != edges.end() ; ++it){
//
//			std::vector<Eigen::Vector3d> points;
//
//			for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
//				geometry_msgs::Point p;
//				g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
//				auto vertex = ptr->estimate().toVector();
//				vertex[2] = acg.getZElevation();
//				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
////				Eigen::Vector2d veve; veve << vertex(0), vertex(1);
////                 std::cout << "Pushing " << veve << std::endl;
//				points.push_back(vertex);
//			}
//			assert(points.size() == 2);
//
//			Eigen::Vector3d slope = points[1] - points[0];
////			std::cout << "from " << points[0] << " tot " << points[1] << " slope " << slope << std::endl;
//
//
//			slope = slope / slope.norm();
////			std::cout << "Then slope " << slope << std::endl;
//			slope = slope * resolution;
////			std::cout << "Final slope " << slope << std::endl;
//
////			int wait;
////			std::cin>>wait;
//
//			Eigen::Vector3d point = points[0];
//
//			pcl::PointXYZ pcl_point;
//			pcl_point.x = point[0];
//			pcl_point.y = point[1];
//			pcl_point.z = point[2];
//			pcl_pc->push_back(pcl_point);
//			nb_points++;
//
//			while( (points[1] - point).norm() >= resolution) {
//
////				std::cout << "Adding point " << pcl_point.x << " " << pcl_point.y << " " << pcl_point.z << " nbpt " << nb_points << " "  << slope[0] << " " << slope[1] << " " << slope[2] << std::endl;
//				point = point + slope;
//				pcl_point.x = point[0];
//				pcl_point.y = point[1];
//				pcl_point.z = point[2];
//				pcl_pc->push_back(pcl_point);
//				nb_points++;
//
//			}
//
//		}
//
//		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_noise(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PointXYZ pt;
//		//add some variance on z
//		for(int i=0; i<pcl_pc->points.size(); i++) {
//			pt = pcl_pc->points[i];
//			pt.z += varz*((double)rand())/(double)INT_MAX;
//			pcl_pc_noise->points.push_back(pt);
//
//		}
//
//		std::cout << "Adding data " << nb_points << std::endl;
//
//		pcl_pc_noise->width = nb_points;
//		pcl_pc_noise->height = 1;
//		pcl_pc_noise->is_dense = false;
//
//		return pcl_pc_noise;
//
//	}
//


		template<typename Prior>
	inline void ACGPriorToNDTMap(const Prior& acg, perception_oru::NDTMap& map_out, double z_elevation, double pt_cloud_resolution){

		auto pcl_prior = acg.toPointCloud(pt_cloud_resolution, z_elevation, pt_cloud_resolution/4);

		map_out.loadPointCloud(*pcl_prior);
		map_out.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

		auto allcells = map_out.getAllCellsShared();
		auto allcellsinit = map_out.getAllInitializedCellsShared();
//		std::cout << "all cells " << allcells.size() << " all cells init " << allcellsinit.size() << std::endl;
//		assert(allcells.size() > 0);

	}






	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, grid_map::GridMap& gridMap, double resolution) {

		throw std::runtime_error("DO NOT USE TEMPLATE of prior to grid map");
	}

		template< >
		inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorSE2, g2o::VertexSE2RobotPose, g2o::EdgeSE2Prior_malcolm>& acg, grid_map::GridMap& gridMap, double resolution) {

			auto edges = acg.getPrior()->getEdges();

			gridMap.add("prior");
			gridMap["prior"].setZero();

			auto it = edges.begin();
			for(it ; it != edges.end() ; ++it){


				std::vector<Eigen::Vector2d> points;

				for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
					geometry_msgs::Point p;
					g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
					auto vertex = ptr->estimate().toVector();
					//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
					Eigen::Vector2d veve; veve << vertex(0), vertex(1);
//                 std::cout << "Pushing " << veve << std::endl;
					points.push_back(veve);
				}

				assert(points.size() == 2);

				for (grid_map::LineIterator iterator(gridMap, points[0], points[1]);
				     !iterator.isPastEnd(); ++iterator) {
// 				std::cout << "Stuck" << std::endl;
					gridMap.at("prior", *iterator) = 100;
					// 			publish();
					// 			ros::Duration duration(0.02);
					// 			duration.sleep();
				}
// 			std::cout << "Or not" << std::endl;

			}

// 		return gridMap;

		}

		template< >
		inline void ACGPriortoGridMap(const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>& acg, grid_map::GridMap& gridMap, double resolution) {

			auto edges = acg.getPrior()->getEdges();

			gridMap.add("prior");
			gridMap["prior"].setZero();

			auto it = edges.begin();
			for(it ; it != edges.end() ; ++it){


				std::vector<Eigen::Vector2d> points;

				for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
					geometry_msgs::Point p;
					g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>((*ite2));
					auto vertex = ptr->estimate();
					//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
					Eigen::Vector2d veve; veve << vertex(0), vertex(1);
//                 std::cout << "Pushing " << veve << std::endl;
					points.push_back(veve);
				}

				assert(points.size() == 2);

				for (grid_map::LineIterator iterator(gridMap, points[0], points[1]);
				     !iterator.isPastEnd(); ++iterator) {
// 				std::cout << "Stuck" << std::endl;
					gridMap.at("prior", *iterator) = 100;
					// 			publish();
					// 			ros::Duration duration(0.02);
					// 			duration.sleep();
				}
// 			std::cout << "Or not" << std::endl;

			}

// 		return gridMap;

		}

	///@brief return the biggest absolute value along x and y for the prior.
	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void getPriorSizes(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, double& size_x, double& size_y) {
		throw std::runtime_error("do not use templated version of prior sizes");

	}

	template<>
	inline void getPriorSizes(const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>& acg, double& size_x, double& size_y){
//			throw std::runtime_error("do not use templated version");

		auto edges = acg.getPrior()->getEdges();

		double max_x, min_x, max_y, min_y;
		bool flag_init = false;
		auto it = edges.begin();
		for(it ; it != edges.end() ; ++it){
			for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
				geometry_msgs::Point p;
				g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>((*ite2));
				assert(ptr != NULL);
				auto vertex = ptr->estimate();
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


	}

		template<>
		inline void getPriorSizes(const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorSE2, g2o::VertexSE2RobotPose, g2o::EdgeSE2Prior_malcolm>& acg, double& size_x, double& size_y){
//			throw std::runtime_error("do not use templated version");

			auto edges = acg.getPrior()->getEdges();

			double max_x, min_x, max_y, min_y;
			bool flag_init = false;
			auto it = edges.begin();
			for(it ; it != edges.end() ; ++it){
				for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
					geometry_msgs::Point p;
					g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
					assert(ptr != NULL);
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


		}
	
	
	//TODO : BROKEN FUNCTION FOR NOW
	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGToGridMap(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, grid_map::GridMap& map){
		
        double resol = 0.1;
		
		//************* TODO : shorten this code !
		//Get max sinze of prior
		
// 		auto edges = acg.getEdges();
// 		
// 		double max_x, min_x, max_y, min_y;
// 		bool flag_init = false;
// 		auto it = edges.begin();
// 		for(it ; it != edges.end() ; ++it){
// 			for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
// 				geometry_msgs::Point p;
// 				g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
// 				auto vertex = ptr->estimate().toVector();
// 				if(flag_init == false){
// 					flag_init = true;
// 					max_x = vertex(0);
// 					max_y = vertex(1);
// 					min_x = vertex(0);
// 					min_y = vertex(1);
// 				}
// 				else{
// 					if(max_x < vertex(0)){
// 						max_x = vertex(0);
// 					}
// 					if(max_y < vertex(1)){
// 						max_y = vertex(1);
// 					}
// 					if(min_x > vertex(0)){
// 						min_x = vertex(0);
// 					}
// 					if(min_y > vertex(1)){
// 						min_y = vertex(1);
// 					}
// 				}
// 				
// 			}
// 			
// 		}
// 		
// 		max_x = std::abs(max_x);
// 		max_y = std::abs(max_y);
// 		min_x = std::abs(min_x);
// 		min_y = std::abs(min_y);
		
		double size_x, size_y;
		getPriorSizes<Prior, VertexPrior, EdgePrior>(acg, size_x, size_y);
		
// 		if(max_x > min_x){
// 			size_x = max_x + 10;
// 		}
// 		else{
// 			size_x = min_x + 10;
// 		}
// 		if(max_y > min_y){
// 			size_y = max_y + 10;
// 		} 
// 		else{
// 			size_y = min_y + 10;
// 		}

		/***********************************/
		
// 		grid_map::GridMap map;
		map.setFrameId("/world");
		map.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resol, grid_map::Position(0.0, 0.0));
// 		map.setGeometry(grid_map::Length(1.2, 2.0), 0.4);
		
		map.add("prior"); 
// 		map.add("ndt"); 
// 		map.add("all");
		map["prior"].setZero();
// 		map["ndt"].setZero();
// 		map["all"].setZero();
		
// 		
		ACGPriortoGridMap<Prior, VertexPrior, EdgePrior>(acg, map, resol);
		
// 		grid_map::Matrix& data3 = map["prior"];
// 		for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
// 			const grid_map::Index index(*iterator);
// 			if(std::isnan(data3(index(0), index(1)))){
// 				throw std::runtime_error("FUCK 1 already");
// 			}
// 		}

		
		if(acg.getRobotNodes().size() > 0){
			grid_map::GridMap gridMaptmp({"all"});
			gridMaptmp["all"].setZero();
			
	// 		auto node = acg.getRobotNodes()[0];
	// 		auto vertex = node->estimate().toVector();
			
			gridMaptmp.setFrameId("/world");
			gridMaptmp.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resol, grid_map::Position(0.0, 0.0));
	// 		gridMaptmp.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resol, grid_map::Position(vertex(0), vertex(1)));
		
			nav_msgs::OccupancyGrid* prior_occ = new nav_msgs::OccupancyGrid();
			nav_msgs::OccupancyGrid::ConstPtr ptr_prior_occ(prior_occ);
			
			grid_map::GridMapRosConverter::toOccupancyGrid(gridMaptmp, "all", 0 ,1., *prior_occ);
			
			std::cout << "POsition " << prior_occ->info.origin.position << std::endl;
			std::cout << "ORientation " << prior_occ->info.origin.orientation << std::endl;
	// 		exit(0);
			
			auto occ_out = ACGNDTtoOcc(acg, ptr_prior_occ, resol);
			
	// 		grid_map::Matrix& data4 = map["prior"];
	// 		for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
	// 			const grid_map::Index index(*iterator);
	// 			if(std::isnan(data4(index(0), index(1)))){
	// 				throw std::runtime_error("FUCK before");
	// // 				data2(index(0), index(1)) = -1;
	// 			}
	// 		}
			
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
	// 		
	// 		grid_map::Matrix& data2 = map["prior"];
	// 		for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
	// 			const grid_map::Index index(*iterator);
	// 			if(std::isnan(data2(index(0), index(1)))){
	// 				throw std::runtime_error("FUCK");
	// // 				data2(index(0), index(1)) = -1;
	// 			}
	// 		}
	// 		
	// 		std::cout << map["prior"] << std::endl;
			
	// 		assert(map.getSize()(0) == mapNDT.getSize()(0) && map.getSize()(1) == mapNDT.getSize()(1));
			
	// 		map["all"] = map["prior"] + mapNDT["ndt"];
			
			std::cout << map.getSize()(0) << " " << mapNDT.getSize()(0) << " and " << map.getSize()(1) << " " << mapNDT.getSize()(1) << std::endl;
			std::cout << map.getPosition()(0) << " " << mapNDT.getPosition()(0) << " and " << map.getPosition()(1) << " " << mapNDT.getPosition()(1) << std::endl;
			//Moved because here it's not the same :S ?
	// 		map["all"] = map["prior"].cwiseMax(map["ndt"]);
			grid_map::GridMap gridtmptmp;
	// 		mapNDT.setPosition(map.getPosition());
			
	// 		assert(map.getPosition()(0) == mapNDT.getPosition()(0) && map.getPosition()(1) == mapNDT.getPosition()(1));
			fuseGridMap(mapNDT, map, gridtmptmp, "ndt", "prior");
			
	//		map = gridtmptmp;
			map.add("all");
			map["all"] = gridtmptmp["combined"];
		}
		else{
			map.add("all");
			map["all"] = map["prior"];
		}
	
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
	inline bool toGridMap(perception_oru::NDTMap *ndt_map, grid_map::GridMap &map, double resolution,std::string frame_id, std::string layer_name){//works only for 2D case
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
// 		occ_grid.info.origin.position_in_robot_frame.x=orig_x;
// 		occ_grid.info.origin.position_in_robot_frame.y=orig_y;
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
			perception_oru::NDTCell *cell;
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
// 				perception_oru::NDTCell *cell;
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
	inline void fuseGridMap(const grid_map::GridMap& src, grid_map::GridMap& target, grid_map::GridMap& out_grid, const std::string& layer, const std::string& layer2, const std::string& final_layer){
		
		std::cout << "FUSING " << std::endl;
		assert(target.exists(layer2) == true);
		assert(src.exists(layer) == true);
		
// 		std::cout << "Fusing " << std::endl;
		grid_map::GridMap modifiedMap;
		std::cout << "Resoltuion " << src.getResolution() << " != " << target.getResolution() << std::endl;
// 		if(src.getResolution() != target.getResolution()){
// 			std::cout << "Change res" << std::endl;
// 			grid_map::GridMapCvProcessing::changeResolution(src, modifiedMap, target.getResolution());
// 			std::cout << "Done" << std::endl;
// 		}
// 		else{
			modifiedMap = src;
// 		}
// 		if(target.getSize()(0) == modifiedMap.getSize()(0) && target.getSize()(1) == modifiedMap.getSize()(1)){
// 			target[layer2] = src[layer] + target[layer2];
// 		}
// 		else{
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
			
			std::cout << "MAX : "<< max_x << " " << max_y << std::endl;
			
			out_grid.setFrameId(target.getFrameId());
			// +1 because of non even numbers.
			double res = target.getResolution();
			out_grid.setGeometry(grid_map::Length((max_x * res) + 1, (max_y * res) + 1), target.getResolution(), target.getPosition());
			out_grid.add(final_layer);
			out_grid[final_layer].setZero();
			
			std::cout << "SIZE : " << out_grid.getSize() << " " << grid_map::Length((max_x * res) + 1, (max_y * res) + 1) << std::endl;
			
			
			cv::Mat originalImageP;
			grid_map::GridMapCvConverter::toImage<unsigned short, 1>(out_grid, final_layer, CV_16UC1, 0.0, 1, originalImageP);
			cv::imwrite("/home/malcolm/outgrid_tmp.png", originalImageP);
			
			
			grid_map::Matrix& out = out_grid[final_layer];
			out.setZero();
			
			int t1 = t_size(0);
			int t2 = t_size(1);
			int s1 = s_size(0);
			int s2 = s_size(1);
			
			std::cout << "targ" << (max_x - t_size(0))/2 << " "<< (max_y - t_size(1))/2 << " " << t1 << " " << t2 <<std::endl;
			std::cout << "targ size " << target.getSize() << " compared with " << t1 - ((max_x - t_size(0))/2) << " and " << t2 - ((max_y - t_size(1))/2) << std::endl;
			
			out.block( (max_x - t_size(0))/2, (max_y - t_size(1))/2, t1, t2) = data_targ;
			
			cv::Mat originalImageP1;
			grid_map::GridMapCvConverter::toImage<unsigned short, 1>(out_grid, final_layer, CV_16UC1, 0.0, 1, originalImageP1);
			cv::imwrite("/home/malcolm/outgrid_prior.png", originalImageP1);
			
			std::cout << "src" << (max_x - s_size(0))/2 << " "<< (max_y - s_size(1))/2 << " " << s2 << " " << s2 <<std::endl;
			
			out.block( (max_x - s_size(0))/2, (max_y - s_size(1))/2, s1, s2) = out.block( (max_x - s_size(0))/2, (max_y - s_size(1))/2, s1, s2).cwiseMax(data_src);
			
			
			cv::Mat originalImageP2;
			grid_map::GridMapCvConverter::toImage<unsigned short, 1>(out_grid, final_layer, CV_16UC1, 0.0, 1, originalImageP2);
			cv::imwrite("/home/malcolm/outgrid_all_tog.png", originalImageP2);
			
			std::cout << "assigning target" << std::endl;
// 			target[layer2].resize(max_x, max_y);
// 			target[layer2] = out;
						
// 		}
		
		
		
// 		grid_map::Matrix& data = target[layer2];
// 		
// 		for (grid_map::GridMapIterator it(target); !it.isPastEnd(); ++it) {
// 			grid_map::Position position_in_robot_frame;
// 			target.getPosition(*it, position_in_robot_frame);
// 			try{
// 				double val = modifiedMap.atPosition(layer, position_in_robot_frame);
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
			fuseGridMap(*it, combinedGrid, combinedGrid, layers[i]);
			++i;
// 			cv::Mat originalImageP2fu;
// 			grid_map::GridMapCvConverter::toImage<unsigned short, 1>(combinedGrid, "combined", CV_16UC1, 0.0, 1, originalImageP2fu);
// 			cv::imwrite("/home/malcolm/grid2fusedtemp.png", originalImageP2fu);
// 			exit(0);
		}
		assert(combinedGrid.getSize()(0) == combinedGrid["combined"].rows());
		assert(combinedGrid.getSize()(1) == combinedGrid["combined"].cols());
	}


		template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGToVectorMaps(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, ndt_map::NDTVectorMapMsg& maps){
		if(acg.getRobotNodes().size() != 0){
			for(auto it = acg.getRobotNodes().begin() ; it != acg.getRobotNodes().end(); ++it){
//				std::cout << "ADDING A MAP" << std::endl;
				///Copy map
				ndt_map::NDTMapMsg msg;
				bool good = perception_oru::toMessage((*it)->getMap().get(), msg, "/world");
	// 			
				maps.maps.push_back(msg);
				
				auto pose = (*it)->estimate().toVector();
				geometry_msgs::Pose geo_pose;
				geo_pose.position.x = pose(0);
				geo_pose.position.y = pose(1);
				geo_pose.position.z = acg.getZElevation();
				
				auto quat = tf::createQuaternionFromRPY(0, 0, pose(2));
				geo_pose.orientation.x = quat.getX();
				geo_pose.orientation.y = quat.getY();
				geo_pose.orientation.z = quat.getZ();
				geo_pose.orientation.w = quat.getW();
				
				maps.poses.push_back(geo_pose);

			}
		}
		if(acg.getOdometryEdges().size() != 0 && acg.getRobotNodes().size() >= 2){

			geometry_msgs::Transform trans;
			trans.translation.x = 0;
			trans.translation.y = 0;
			trans.translation.z = 0;

			auto quat = tf::createQuaternionFromRPY(0, 0, 0);
			trans.rotation.x = quat.getX();
			trans.rotation.y = quat.getY();
			trans.rotation.z = quat.getZ();
			trans.rotation.w = quat.getW();

			maps.transformations.push_back(trans);


			for(auto odom : acg.getOdometryEdges()){

				geometry_msgs::Transform trans;
				trans.translation.x = odom->measurement().toVector()(0);
				trans.translation.y = odom->measurement().toVector()(1);
				trans.translation.z = 0;

				auto quat = tf::createQuaternionFromRPY(0, 0, odom->measurement().toVector()(2));
				trans.rotation.x = quat.getX();
				trans.rotation.y = quat.getY();
				trans.rotation.z = quat.getZ();
				trans.rotation.w = quat.getW();

				maps.transformations.push_back(trans);

			}
		}
	}
	
	///@brief transform the ACG into a message including a NDTVectorMapMsg representing all submaps and the transof between them AND the prior represented by grid centered on the origin frame
	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGToACGMapsMsg(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, auto_complete_graph::ACGMaps& mapmsg, const std::string& frame_id = "/world"){
		
		mapmsg.header.stamp = ros::Time::now();
		mapmsg.header.frame_id = frame_id;
		
// 		std::cout << "Vector Map" << std::endl;
		ndt_map::NDTVectorMapMsg maps;
		ACGToVectorMaps(acg, mapmsg.ndt_maps);

		perception_oru::NDTMap* ndt_prior = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
		AASS::acg::ACGPriorToNDTMap<AASS::acg::AutoCompleteGraphPriorXY>(*acg.getPrior(), *ndt_prior, acg.getZElevation(), 0.1);

		ndt_map::NDTMapMsg mapmsgndt;
		perception_oru::toMessage(ndt_prior, mapmsgndt, frame_id);

		mapmsg.prior = mapmsgndt;
		
// 		std::cout << "Grid Map" << std::endl;
//		grid_map::GridMap gridMap;
//		gridMap.setFrameId("/world");
//		double size_x, size_y;
//		getPriorSizes<Prior, VertexPrior, EdgePrior>(acg, size_x, size_y);
//		gridMap.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), 0.1, grid_map::Position(0.0, 0.0));
//		gridMap.add("prior");
//		gridMap["prior"].setZero();
//		double resolution = 0.1;
//		ACGPriortoGridMap<Prior, VertexPrior, EdgePrior>(acg, gridMap, resolution);
//		grid_map::GridMapRosConverter converter;
//		grid_map_msgs::GridMap gridmapmsg;
//		converter.toMessage(gridMap, mapmsg.prior);
		
		
	}




		template< typename Prior, typename VertexPrior, typename EdgePrior>
		inline void ACGToOccMaps(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, auto_complete_graph::ACGMapsOM& mapmsg, double resolution = 0.1, const std::string& frame_id = "/world", double ndt_cell_gaussian_scaling = 0.1, bool euclidean_dist_occ_map = false){
//		if(acg.getRobotNodes().size() != 0){
			for(auto it = acg.getRobotNodes().begin() ; it != acg.getRobotNodes().end(); ++it){

// 				grid_map::GridMap gridMaptmp({"ndt"});
// 				gridMaptmp["ndt"].setZero();
//
// 		// 		auto node = acg.getRobotNodes()[0];
// 		// 		auto vertex = node->estimate().toVector();
//
// 				gridMaptmp.setFrameId("/world");
// 				double size_x, size_y;
// 				getPriorSizes(acg, size_x, size_y);
// 				gridMaptmp.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resolution, grid_map::Position(0.0, 0.0));

				nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();
// 					initOccupancyGrid(*omap, 250, 250, 0.4, "/world");

//				std::cout << "OTHER Resolution " << resolution << " scaling " << ndt_cell_gaussian_scaling << std::endl;
//				exit(0);
				perception_oru::toOccupancyGrid((*it)->getMap().get(), *omap, resolution, frame_id, ndt_cell_gaussian_scaling, euclidean_dist_occ_map);

				grid_map::GridMap mapNDT;
				//THis ruin prior because they are of different sizes ! Need my custom fuse function :)
				grid_map::GridMapRosConverter::fromOccupancyGrid(*omap, "ndt", mapNDT);
				delete omap;

//				grid_map::Matrix& data = mapNDT["ndt"];
//				for (grid_map::GridMapIterator iterator(mapNDT); !iterator.isPastEnd(); ++iterator) {
//					const grid_map::Index index(*iterator);
//					if(std::isnan(data(index(0), index(1)))){
//						data(index(0), index(1)) = -1;
//					}
//				}

// 				std::cout << "ADDING A MAP" << std::endl;
// 				///Copy map
// 				ndt_map::NDTMapMsg msg;
// 				bool good = perception_oru::toMessage((*it)->getMap().get(), msg, "/world");
				grid_map::GridMapRosConverter converter;
				grid_map_msgs::GridMap gridmapmsg;
				converter.toMessage(mapNDT, gridmapmsg);

// 				std::cout << "Layers " << std::endl;
// 				for(int i  = 0; i < gridmapmsg.layers.size() ; ++i){
// 					std::cout << gridmapmsg.layers[i] << std::endl;
// 				}
//
// 				std::cout << "Layers Basic" << std::endl;
// 				for(int i  = 0; i < gridmapmsg.basic_layers.size() ; ++i){
// 					std::cout << gridmapmsg.basic_layers[i] << std::endl;
// 				}
				//
				mapmsg.ndt_maps_om.push_back(gridmapmsg);

				auto pose = (*it)->estimate().toVector();
				geometry_msgs::Transform transform;
				transform.translation.x = pose(0);
				transform.translation.y = pose(1);
				transform.translation.z = 0;

				auto quat = tf::createQuaternionFromRPY(0, 0, pose(2));
				transform.rotation.x = quat.getX();
				transform.rotation.y = quat.getY();
				transform.rotation.z = quat.getZ();
				transform.rotation.w = quat.getW();

				mapmsg.robot_poses.push_back(transform);

			}
//		}
		}


//		//THAT IS UGLY BUT I NEED IT FAST :( LOCALIZATION AND ROBOT POSE SHOULD BE THE SAME THING
//		inline void ACGToOccMaps(const AASS::acg::AutoCompleteGraphLocalization& acg, auto_complete_graph::ACGMapsOM& mapmsg, double resolution, const std::string& frame_id = "/world", double ndt_cell_gaussian_scaling = 0.05){
////		if(acg.getRobotNodes().size() != 0){
//			for(auto it = acg.getRobotPoseLocalization().begin() ; it != acg.getRobotPoseLocalization().end(); ++it){
//
//// 				grid_map::GridMap gridMaptmp({"ndt"});
//// 				gridMaptmp["ndt"].setZero();
////
//// 		// 		auto node = acg.getRobotNodes()[0];
//// 		// 		auto vertex = node->estimate().toVector();
////
//// 				gridMaptmp.setFrameId("/world");
//// 				double size_x, size_y;
//// 				getPriorSizes(acg, size_x, size_y);
//// 				gridMaptmp.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resolution, grid_map::Position(0.0, 0.0));
//
//				nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();
//// 					initOccupancyGrid(*omap, 250, 250, 0.4, "/world");
//
////				std::cout << "Resolution " << resolution << " scaling " << ndt_cell_gaussian_scaling << std::endl;
////				exit(0);
//				perception_oru::toOccupancyGrid((*it)->getMap().get(), *omap, resolution, frame_id, ndt_cell_gaussian_scaling);
//
//				grid_map::GridMap mapNDT;
//				//THis ruin prior because they are of different sizes ! Need my custom fuse function :)
//				grid_map::GridMapRosConverter::fromOccupancyGrid(*omap, "ndt", mapNDT);
//				delete omap;
//
//				grid_map::Matrix& data = mapNDT["ndt"];
//				for (grid_map::GridMapIterator iterator(mapNDT); !iterator.isPastEnd(); ++iterator) {
//					const grid_map::Index index(*iterator);
//					if(std::isnan(data(index(0), index(1)))){
//						data(index(0), index(1)) = -1;
//					}
//				}
//
//// 				std::cout << "ADDING A MAP" << std::endl;
//// 				///Copy map
//// 				ndt_map::NDTMapMsg msg;
//// 				bool good = perception_oru::toMessage((*it)->getMap().get(), msg, "/world");
//				grid_map::GridMapRosConverter converter;
//				grid_map_msgs::GridMap gridmapmsg;
//				converter.toMessage(mapNDT, gridmapmsg);
//
//// 				std::cout << "Layers " << std::endl;
//// 				for(int i  = 0; i < gridmapmsg.layers.size() ; ++i){
//// 					std::cout << gridmapmsg.layers[i] << std::endl;
//// 				}
////
//// 				std::cout << "Layers Basic" << std::endl;
//// 				for(int i  = 0; i < gridmapmsg.basic_layers.size() ; ++i){
//// 					std::cout << gridmapmsg.basic_layers[i] << std::endl;
//// 				}
//				//
//				mapmsg.ndt_maps_om.push_back(gridmapmsg);
//
//				auto pose = (*it)->estimate().toVector();
//				geometry_msgs::Transform transform;
//				transform.translation.x = pose(0);
//				transform.translation.y = pose(1);
//				transform.translation.z = 0;
//
//				auto quat = tf::createQuaternionFromRPY(0, 0, pose(2));
//				transform.rotation.x = quat.getX();
//				transform.rotation.y = quat.getY();
//				transform.rotation.z = quat.getZ();
//				transform.rotation.w = quat.getW();
//
//				mapmsg.robot_poses.push_back(transform);
//
//			}
////		}
//		}


	///@brief transform the ACG into a message including a NDTVectorMapMsg representing all submaps and the transof between them AND the prior represented by grid centered on the origin frame
	template< typename Prior, typename VertexPrior, typename EdgePrior>
	inline void ACGToACGMapsOMMsg(const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg, auto_complete_graph::ACGMapsOM& mapmsg, const std::string& frame_id = "/world", double resolution = 0.1, double ndt_cell_gaussian_scaling = 0.05, bool euclidean_dist_occ_map = false){
		
		mapmsg.header.stamp = ros::Time::now();
		mapmsg.header.frame_id = frame_id;
		
// 		std::cout << "Vector Map" << std::endl;
// 		ndt_map::NDTVectorMapMsg maps;
		ACGToOccMaps(acg, mapmsg, resolution, frame_id, ndt_cell_gaussian_scaling, euclidean_dist_occ_map);
		
// 		std::cout << "Grid Map" << std::endl;
		grid_map::GridMap gridMap;
		gridMap.setFrameId(frame_id);
		double size_x, size_y;
		getPriorSizes(acg, size_x, size_y);
		gridMap.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resolution, grid_map::Position(0.0, 0.0));
		gridMap.add("prior"); 
		gridMap["prior"].setZero(); 
//		double resolution = 0.1;
		ACGPriortoGridMap(acg, gridMap, resolution);
		grid_map::GridMapRosConverter converter;
		grid_map_msgs::GridMap gridmapmsg;
		converter.toMessage(gridMap, mapmsg.prior);
		
		
	}
	
	
}
}


#endif
