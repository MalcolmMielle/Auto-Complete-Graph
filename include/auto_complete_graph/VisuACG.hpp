#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016

#include "auto_complete_graph/ACG.hpp"
#include "occupancy_grid_utils/combine_grids.h"

#include "acg_conversion.hpp"


namespace AASS {

namespace acg{	
	
	
	class VisuAutoCompleteGraph{ 
	
	protected:
		ros::NodeHandle _nh;
		ros::Publisher _last_ndtmap;
		ros::Publisher _last_ndtmap2;
		ros::Publisher _last_ndtmap_full;
		ros::Publisher _marker_pub;
		ros::Publisher _ndt_node_pub;
		ros::Publisher _prior_node_pub;
		ros::Publisher _corner_ndt_node_pub;
		ros::Publisher _link_pub;
// 		nav_msgs::OccupancyGrid omap;
		int _nb_of_zone;
		AutoCompleteGraph* _acg;
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids_original;
		visualization_msgs::Marker _prior_edge_markers;
		visualization_msgs::Marker _ndt_node_markers;
		visualization_msgs::Marker _prior_node_markers;
		visualization_msgs::Marker _corner_ndt_node_markers;
		visualization_msgs::Marker _link_markers;
		
		double _resolution;
		
		std::string _image_file;
		
	public:
		VisuAutoCompleteGraph(AutoCompleteGraph* acg, ros::NodeHandle nh) : _nb_of_zone(0), _resolution(0.1){
			_nh = nh;
			_last_ndtmap = _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg", 10);
			_last_ndtmap2 = _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg2", 10);
			_last_ndtmap_full = _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg_full", 10);
			
			_marker_pub = _nh.advertise<visualization_msgs::Marker>("prior_marker", 10);
			_ndt_node_pub = _nh.advertise<visualization_msgs::Marker>("ndt_nodes_marker", 10);
			_prior_node_pub = _nh.advertise<visualization_msgs::Marker>("prior_nodes_marker", 10);
			_corner_ndt_node_pub = _nh.advertise<visualization_msgs::Marker>("corner_ndt_marker", 10);
			_link_pub = _nh.advertise<visualization_msgs::Marker>("link_markers", 10);
			
			_acg = acg;
			
			_prior_edge_markers.type = visualization_msgs::Marker::LINE_LIST;
			_prior_edge_markers.header.frame_id = "/world";
			_prior_edge_markers.ns = "acg";
			_prior_edge_markers.id = 0;
			_prior_edge_markers.scale.x = 0.2;
			_prior_edge_markers.scale.y = 0.2;
			_prior_edge_markers.color.b = 1.0f;
			_prior_edge_markers.color.a = 1.0;
			
			_ndt_node_markers.type = visualization_msgs::Marker::POINTS;
			_ndt_node_markers.header.frame_id = "/world";
			_ndt_node_markers.ns = "acg";
			_ndt_node_markers.id = 1;
			_ndt_node_markers.scale.x = 0.2;
			_ndt_node_markers.scale.y = 0.2;
			_ndt_node_markers.color.r = 1.0f;
			_ndt_node_markers.color.a = 1.0;
			
			_prior_node_markers.type = visualization_msgs::Marker::POINTS;
			_prior_node_markers.header.frame_id = "/world";
			_prior_node_markers.ns = "acg";
			_prior_node_markers.id = 1;
			_prior_node_markers.scale.x = 0.5;
			_prior_node_markers.scale.y = 0.5;
			_prior_node_markers.color.r = 0.5f;
			_prior_node_markers.color.a = 1.0;
			
			_corner_ndt_node_markers.type = visualization_msgs::Marker::POINTS;
			_corner_ndt_node_markers.header.frame_id = "/world";
			_corner_ndt_node_markers.ns = "acg";
			_corner_ndt_node_markers.id = 2;
			_corner_ndt_node_markers.scale.x = 0.5;
			_corner_ndt_node_markers.scale.y = 0.5;
			_corner_ndt_node_markers.color.g = 1.0f;
			_corner_ndt_node_markers.color.a = 1.0;
			
			_link_markers.type = visualization_msgs::Marker::LINE_LIST;
			_link_markers.header.frame_id = "/world";
			_link_markers.ns = "acg";
			_link_markers.id = 3;
			_link_markers.scale.x = 0.2;
			_link_markers.scale.y = 0.2;
			_link_markers.color.g = 1.0f;
			_link_markers.color.r = 1.0f;
			_link_markers.color.a = 1.0;
// 			initOccupancyGrid(omap, 500, 500, 0.4, "/world");
		}
// 		void toRviz(const AutoCompleteGraph& acg);
		
		void setImageFileNameOut(const std::string& f){_image_file = f;}
		
		
		void updateRvizStepByStep(){
			
			drawPrior();
			
			drawCornersNdt();
			
			drawLinks();
			
// 			_ndt_node_markers.points.clear();
			
			if(_nb_of_zone != _acg->getRobotNodes().size()){
				nav_msgs::OccupancyGrid* omap_tmpt = new nav_msgs::OccupancyGrid();
				nav_msgs::OccupancyGrid::Ptr occ_outt(omap_tmpt);
				ACGtoOccupancyGrid(*_acg, occ_outt, _acg->getRobotNodes().size() - 2);
				
// 				grid_map::GridMap gridMap;
// 				ACGToGridMap(*_acg, gridMap);
				
				std::cout << "Going to publish" << std::endl;
				
// 				Eigen::Affine2d aff; aff.matrix() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
// 				moveOccupancyMap(*occ_out, aff);
// 				
// 				grid_map::GridMap gridMap({"all"});
// 				grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_out, "all", gridMap);
				
// 				std::cout << "To occ" << std::endl;
// 				nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();
// 				nav_msgs::OccupancyGrid::Ptr occ_out(omap_tmp);
// 				grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "all", 0, 1, *occ_out);
				
				std::cout << "WELLL HERE IT IS : " << occ_outt->info.origin.position << " ori " << occ_outt->info.origin.orientation << std::endl << std::endl;
// 				std::cout << "WELLL HERE IT IS : " << occ_out->info.origin.position << " ori " << occ_out->info.origin.orientation << std::endl << std::endl;
				
// 				exit(0);
				
// 				auto node = _acg->getRobotNodes()[0].getNode();
// 				auto vertex = node->estimate().toIsometry();
// 				moveOccupancyMap(*occ_out, vertex);
				
				
// 				cv::Mat originalImageP;
// 				grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "all", CV_16UC1, 0.0, 1, originalImageP);
// 				cv::imwrite("/home/malcolm/tmp_all.png", originalImageP);
				
				std::cout << "Pub" << std::endl;
				_last_ndtmap_full.publish<nav_msgs::OccupancyGrid>(*occ_outt);
// 				saveImage(occ_out);
// 				std::cout << "Image saved" << std::endl;
				
// 				_last_ndtmap.publish<nav_msgs::OccupancyGrid>(*occ_out);
				
// 				exit(0);
				
			}
		}
		
		void updateRviz(){
			
			drawPrior();
			
			drawCornersNdt();
			
			drawLinks();
			
// 			_ndt_node_markers.points.clear();
			
			if(_nb_of_zone != _acg->getRobotNodes().size()){
				nav_msgs::OccupancyGrid* omap_tmpt = new nav_msgs::OccupancyGrid();
				nav_msgs::OccupancyGrid::Ptr occ_outt(omap_tmpt);
				ACGtoOccupancyGrid(*_acg, occ_outt);
				
// 				grid_map::GridMap gridMap;
// 				ACGToGridMap(*_acg, gridMap);
				
// 				std::cout << "Going to publish" << std::endl;
				
// 				Eigen::Affine2d aff; aff.matrix() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
// 				moveOccupancyMap(*occ_out, aff);
// 				
// 				grid_map::GridMap gridMap({"all"});
// 				grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_out, "all", gridMap);
				
// 				std::cout << "To occ" << std::endl;
// 				nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();
// 				nav_msgs::OccupancyGrid::Ptr occ_out(omap_tmp);
// 				grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "all", 0, 1, *occ_out);
				
// 				std::cout << "WELLL HERE IT IS : " << occ_outt->info.origin.position << " ori " << occ_outt->info.origin.orientation << std::endl << std::endl;
// 				std::cout << "WELLL HERE IT IS : " << occ_out->info.origin.position << " ori " << occ_out->info.origin.orientation << std::endl << std::endl;
				
// 				exit(0);
				
// 				auto node = _acg->getRobotNodes()[0].getNode();
// 				auto vertex = node->estimate().toIsometry();
// 				moveOccupancyMap(*occ_out, vertex);
				
				
// 				cv::Mat originalImageP;
// 				grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "all", CV_16UC1, 0.0, 1, originalImageP);
// 				cv::imwrite("/home/malcolm/tmp_all.png", originalImageP);
				
// 				std::cout << "Pub" << std::endl;
				_last_ndtmap_full.publish<nav_msgs::OccupancyGrid>(*occ_outt);
// 				saveImage(occ_out);
// 				std::cout << "Image saved" << std::endl;
				
// 				_last_ndtmap.publish<nav_msgs::OccupancyGrid>(*occ_out);
				
// 				exit(0);
				
			}
		}
		
		void updateRvizNoNDT(){
			
			drawPrior();
			
			drawCornersNdt();
			
			drawLinks();
			
// 			_ndt_node_markers.points.clear();
			
// 			if(_nb_of_zone != _acg->getRobotNodes().size()){
// 				nav_msgs::OccupancyGrid* omap_tmpt = new nav_msgs::OccupancyGrid();
// 				nav_msgs::OccupancyGrid::Ptr occ_outt(omap_tmpt);
// 				ACGtoOccupancyGrid(*_acg, occ_outt);
// 				
// // 				grid_map::GridMap gridMap;
// // 				ACGToGridMap(*_acg, gridMap);
// 				
// 				std::cout << "Going to publish" << std::endl;
// 				
// // 				Eigen::Affine2d aff; aff.matrix() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
// // 				moveOccupancyMap(*occ_out, aff);
// // 				
// // 				grid_map::GridMap gridMap({"all"});
// // 				grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_out, "all", gridMap);
// 				
// // 				std::cout << "To occ" << std::endl;
// // 				nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();
// // 				nav_msgs::OccupancyGrid::Ptr occ_out(omap_tmp);
// // 				grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "all", 0, 1, *occ_out);
// 				
// 				std::cout << "WELLL HERE IT IS : " << occ_outt->info.origin.position << " ori " << occ_outt->info.origin.orientation << std::endl << std::endl;
// // 				std::cout << "WELLL HERE IT IS : " << occ_out->info.origin.position << " ori " << occ_out->info.origin.orientation << std::endl << std::endl;
// 				
// // 				exit(0);
// 				
// // 				auto node = _acg->getRobotNodes()[0].getNode();
// // 				auto vertex = node->estimate().toIsometry();
// // 				moveOccupancyMap(*occ_out, vertex);
// 				
// 				
// // 				cv::Mat originalImageP;
// // 				grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "all", CV_16UC1, 0.0, 1, originalImageP);
// // 				cv::imwrite("/home/malcolm/tmp_all.png", originalImageP);
// 				
// 				std::cout << "Pub" << std::endl;
// 				_last_ndtmap_full.publish<nav_msgs::OccupancyGrid>(*occ_outt);
// // 				saveImage(occ_out);
// // 				std::cout << "Image saved" << std::endl;
// 				
// // 				_last_ndtmap.publish<nav_msgs::OccupancyGrid>(*occ_out);
// 				
// // 				exit(0);
// 				
// 			}
		}
		
		
		void updateRvizV2(){
			
			drawPrior();
			
			drawCornersNdt();
			
			drawLinks();
			
// 			_ndt_node_markers.points.clear();
			
			
			if(_nb_of_zone != _acg->getRobotNodes().size()){
				grids.clear();
				std::cout <<"update the zones" << std::endl;
				
				for(size_t i = 0 ; i < _acg->getRobotNodes().size() ; ++i){
// 				for(size_t i = 0 ; i < 1 ; ++i){
					
					
					
// 					exit(0);
					///Cheating here for quick visualization purpose
					
//Grid map test
// 					grid_map::GridMap gridMap({"elevation"});
// 					std::cout << "Converting" << std::endl;
// 					toGridMap(_acg->getRobotNodes()[i].getMap(), gridMap, 0.4, "/world", "elevation");
// 					cv_bridge::CvImage cvImage;
// 					grid_map::GridMapRosConverter converter;
// 					
// 					std::cout << "To openCV" << std::endl;
// 					cv::Mat originalImage;
// 					grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "elevation", CV_16UC1, 0.0, 0.3, originalImage);
// // 					bool res = converter.toCvImage(gridMap, "elevation", CV_8U, cvImage);
// // 					if(res == true){
// 						cv::imwrite("/home/malcolm/tmp.png", originalImage);
// // 						std::cout << "Image written" << std::endl;
// // 					}
// // 					else{
// // 						std::runtime_error("FUCK");
// // 					}
// 					std::cout << "Prior" << std::endl;
// 						
// // 					grid_map::GridMap gridPrior({"prior"});
// 					grid_map::GridMap gridPrior;
// 					ACGToGridMap(*_acg, gridPrior);
// // 					ACGPriortoGridMap(*_acg, 0.4);
// 					
// 					cv::Mat originalImageP;
// 					grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridPrior, "all", CV_16UC1, 0.0, 1, originalImageP);
// // 					bool res = converter.toCvImage(gridMap, "elevation", CV_8U, cvImage);
// // 					if(res == true){
// 					cv::imwrite("/home/malcolm/tmp_all.png", originalImageP);
// 					
// 					cv::Mat originalImagePp;
// 					grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridPrior, "prior", CV_16UC1, 0.0, 1, originalImagePp);
// // 					bool res = converter.toCvImage(gridMap, "elevation", CV_8U, cvImage);
// // 					if(res == true){
// 					cv::imwrite("/home/malcolm/tmp_prior.png", originalImagePp);
// 					
// 					cv::Mat originalImagePn;
// 					grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridPrior, "ndt", CV_16UC1, 0.0, 1, originalImagePn);
// // 					bool res = converter.toCvImage(gridMap, "elevation", CV_8U, cvImage);
// // 					if(res == true){
// 					cv::imwrite("/home/malcolm/tmp_ndt.png", originalImagePn);
						
					
					
					/*************************/
					
					nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();			
// 					initOccupancyGrid(*omap_tmp, 250, 250, 0.4, "/world");
					lslgeneric::toOccupancyGrid(_acg->getRobotNodes()[i].getMap(), *omap_tmp, _resolution, "/world");
// 					auto pose = _acg->getRobotNodes()[i].getPose();
					auto node = _acg->getRobotNodes()[i].getNode();
					auto vertex = node->estimate().toIsometry();
// 					Eigen::Vector3d vector; vector << vertex(0), vertex(1), vertex(2);
// 					std::cout << "Move : " << node.matrix() << std::endl;
// 					if(i == 2) exit(0);
					moveOccupancyMap(*omap_tmp, vertex);
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
					
					
					geometry_msgs::Point p;
					auto vertex2 = node->estimate().toVector();
					//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
					p.x = vertex2(0);
					p.y = vertex2(1);
					p.z = 0;
					
					_ndt_node_markers.points.push_back(p);
					
				}
				
			}
			
// 			omap.header.frame_id = "/world";
// 			omap.header.stamp = ros::Time::now();
			nav_msgs::OccupancyGrid::Ptr final;
			if(grids.size() > 0){
				
				std::cout << "Combining " << grids.size() << std::endl;
				final = occupancy_grid_utils::combineGrids(grids);
// 				std::cout << "Ref frame " << omap.header.frame_id << std::endl;
				final->header.frame_id = "/world";
				final->header.stamp = ros::Time::now();
				_last_ndtmap.publish<nav_msgs::OccupancyGrid>(*final);
				
				_ndt_node_pub.publish(_ndt_node_markers);
// 				if(_nb_of_zone != _acg->getRobotNodes().size()){
// 					saveImage(final);
// 				}
// 				std::cout <<"Final orientaiotn " << final->info.origin.orientation << std::endl;
// 				exit(0);
			}
			
			_nb_of_zone = _acg->getRobotNodes().size();
			
			
			
			
			
			
		}
		
		void updateRvizOriginalSLAM(){
			
			drawPrior();
			drawLinks();
			
			if(_nb_of_zone != _acg->getRobotNodes().size()){
				
				std::cout <<"update the zones" << std::endl;
				
				for(size_t i = _nb_of_zone ; i < _acg->getRobotNodes().size() ; ++i){
// 				for(size_t i = 0 ; i < 1 ; ++i){
					
					nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();			
// 					initOccupancyGrid(*omap_tmp, 250, 250, 0.4, "/world");
					lslgeneric::toOccupancyGrid(_acg->getRobotNodes()[i].getMap(), *omap_tmp, _resolution, "/world");
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
		void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine2d &pose_vec);
// 		void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Vector3d &pose_vec);
		
		//TO TEST
		Eigen::Affine3d vector3dToAffine3d(const Eigen::Vector3d& vec){
			Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));
			return t;
		}
		
		void drawPrior();
		void drawLinks();
		void drawCornersNdt();
		
		void saveImage(nav_msgs::OccupancyGrid::Ptr& msg);
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
	
	inline void AASS::acg::VisuAutoCompleteGraph::moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine2d &a2d) {

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
	


	inline void VisuAutoCompleteGraph::drawPrior()
	{
		_prior_edge_markers.header.stamp = ros::Time::now();
		auto edges = _acg->getPriorEdges();
		if(edges.size() != _prior_edge_markers.points.size()){
			_prior_edge_markers.points.clear();
			auto it = edges.begin();
			for(it ; it != edges.end() ; ++it){
				for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
					geometry_msgs::Point p;
					g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
					auto vertex = ptr->estimate().toVector();
					//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
					p.x = vertex(0);
					p.y = vertex(1);
					p.z = 0;
					_prior_edge_markers.points.push_back(p);
				}
			}
			
			auto prior_node = _acg->getPriorNodes();
			_prior_node_markers.points.clear();
			auto itt = prior_node.begin();
			for(itt ; itt != prior_node.end() ; ++itt){
				
				geometry_msgs::Point p;
				g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>((*itt));
				auto vertex = ptr->estimate().toVector();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = 0;
				_prior_node_markers.points.push_back(p);
				
			}
		}
		_marker_pub.publish(_prior_edge_markers);
		_prior_node_pub.publish(_prior_node_markers);
	}
	
	inline void VisuAutoCompleteGraph::drawLinks()
	{
		_link_markers.header.stamp = ros::Time::now();
		auto edges = _acg->getLinkEdges();
		if(edges.size() != _link_markers.points.size()){
			_link_markers.points.clear();
			auto it = edges.begin();
			for(it ; it != edges.end() ; ++it){
				for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
					
					geometry_msgs::Point p;
				
					g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
					g2o::VertexPointXY* ptr2 = dynamic_cast<g2o::VertexPointXY*>((*ite2));
					
					if(ptr != NULL){
// 						std::cout << "Got a VertexSE2" << std::endl;
						auto vertex = ptr->estimate().toVector();
						p.x = vertex(0);
						p.y = vertex(1);
						p.z = 0;
					}
					else if(ptr2 != NULL){
// 						std::cout << "Got a VertexPOINTXY" << std::endl;
						auto vertex = ptr2->estimate();
						p.x = vertex(0);
						p.y = vertex(1);
						p.z = 0;
					}
					else{
						throw std::runtime_error("Links do not have the good vertex type");
					}			
					
					_link_markers.points.push_back(p);
				}
			}
		}
		_link_pub.publish(_link_markers);
	}
	
	inline void VisuAutoCompleteGraph::drawCornersNdt()
	{
// 		std::cout << "Getting the corners" << std::endl;
		_corner_ndt_node_markers.header.stamp = ros::Time::now();
		auto edges = _acg->getLandmarkNodes();
// 		std::cout << "Getting the corners " << edges.size() << std::endl;
		if(edges.size() != _corner_ndt_node_markers.points.size()){
			_corner_ndt_node_markers.points.clear();
			auto it = edges.begin();
			for(it ; it != edges.end() ; ++it){
				
				geometry_msgs::Point p;
				g2o::VertexPointXY* ptr = dynamic_cast<g2o::VertexPointXY*>((*it));
				auto vertex = ptr->estimate();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = 0;
				_corner_ndt_node_markers.points.push_back(p);
				
			}
		}
		_corner_ndt_node_pub.publish(_corner_ndt_node_markers);
	}
	
	
	inline void VisuAutoCompleteGraph::saveImage(nav_msgs::OccupancyGrid::Ptr& msg)
	{
// 		const unsigned char* databeg = &(msg->data.front());
		
		assert(msg->info.height*msg->info.width == msg->data.size());
		
		std::cout << "Creating the mat" << std::endl;
// 		cv::Mat img = cv::Mat(msg->info.height, msg->info.width, CV_32SC1, &(msg->data[0])) ;
		cv::Mat img = cv::Mat(msg->data, true) ;
		img.rows = msg->info.height;
		img.cols = msg->info.width;
		
		std::string file_out = _image_file;
		std::ostringstream convert;   // stream used for the conversion
		convert << _acg->getGraph().vertices().size(); 
		file_out = file_out + convert.str();
		file_out = file_out + "nodes.png";
		
// 		cv::imshow("Show", img);
// 		cv::waitKey(0);
		
		std::cout << "IMwrite to " << file_out << std::endl;
		cv::imwrite(file_out, img);

	}


	
	
}
}

#endif