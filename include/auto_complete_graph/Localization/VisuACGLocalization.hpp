#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018

#include "auto_complete_graph/VisuACG.hpp"
#include "auto_complete_graph/Localization/ACG_localization.hpp"

#include "auto_complete_graph/utils.hpp"

namespace AASS {

	namespace acg {


		class VisuAutoCompleteGraphLocalization : public VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG> {

		protected:

			visualization_msgs::Marker _localization_edge_markers;
			visualization_msgs::Marker _prior_observations;
			visualization_msgs::Marker _ndt_node_localization_markers;
			visualization_msgs::Marker _mcl_angles_markers;
			visualization_msgs::Marker _robot_pose_angles_markers;
			visualization_msgs::Marker _last_seen_landmark_in_mcl_pose;
			visualization_msgs::Marker _ndt_cells;
			visualization_msgs::Marker _ndt_cells_mcl;
			visualization_msgs::Marker _ndt_cell_observations;
			visualization_msgs::Marker _ndt_cell_associations;
			visualization_msgs::Marker _correct_robot_pose;
			visualization_msgs::Marker _mcl_poses;
			ros::Publisher _localization_pub;
			ros::Publisher _localization_pose_pub;
			ros::Publisher _prior_observations_pub;
			ros::Publisher _mcl_angles_pub;
			ros::Publisher _robot_pose_angles_pub;
			ros::Publisher _last_landmark;
			ros::Publisher _mcl_localization;
			ros::Publisher _ndt_cell_pub;
			ros::Publisher _mcl_ndt_cell_pub;
			ros::Publisher _ndt_cell_observation_pub;
			ros::Publisher _ndt_cell_association_pub;
			ros::Publisher _correct_robot_pose_pub;


//			AutoCompleteGraphLocalization *_acg;


		public:
			VisuAutoCompleteGraphLocalization(const ros::NodeHandle &nh, const std::string& world_frame_id = "/world")
					: VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>(nh, world_frame_id) {

//				_acg = acg;

				_localization_pub = _nh.advertise<visualization_msgs::Marker>("localization_markers", 10);
				_localization_pose_pub = _nh.advertise<visualization_msgs::Marker>("localization_pose_markers", 10);
				_prior_observations_pub = _nh.advertise<visualization_msgs::Marker>("prior_observations_markers", 10);
				_mcl_angles_pub = _nh.advertise<visualization_msgs::Marker>("mcl_angles_markers", 10);
				_robot_pose_angles_pub = _nh.advertise<visualization_msgs::Marker>("robot_pose_angles_markers", 10);
				_last_landmark = _nh.advertise<visualization_msgs::Marker>("mcl_landmark", 10);
				_mcl_localization = _nh.advertise<visualization_msgs::Marker>("mcl_localization", 10);
				_ndt_cell_pub = _nh.advertise<visualization_msgs::Marker>("ndt_cell", 10);
				_mcl_ndt_cell_pub = _nh.advertise<visualization_msgs::Marker>("mcl_ndt_cell", 10);
				_ndt_cell_observation_pub = _nh.advertise<visualization_msgs::Marker>("ndt_cell_observations", 10);
				_ndt_cell_association_pub = _nh.advertise<visualization_msgs::Marker>("ndt_cell_associations", 10);
				_correct_robot_pose_pub = _nh.advertise<visualization_msgs::Marker>("initial_robot_pose", 10);

				_localization_edge_markers.type = visualization_msgs::Marker::LINE_LIST;
				_localization_edge_markers.header.frame_id = world_frame_id;
				_localization_edge_markers.ns = "acg";
				_localization_edge_markers.id = 0;
				_localization_edge_markers.scale.x = 0.1;
				_localization_edge_markers.scale.y = 0.1;
				_localization_edge_markers.color.b = 1.0f;
				_localization_edge_markers.color.a = 1.0;


				_prior_observations.type = visualization_msgs::Marker::LINE_LIST;
				_prior_observations.header.frame_id = world_frame_id;
				_prior_observations.ns = "acg";
				_prior_observations.id = 0;
				_prior_observations.scale.x = 0.1;
				_prior_observations.scale.y = 0.1;
				_prior_observations.color.b = 1.0f;
				_prior_observations.color.a = 1.0;

				_last_seen_landmark_in_mcl_pose.type = visualization_msgs::Marker::POINTS;
				_last_seen_landmark_in_mcl_pose.header.frame_id = world_frame_id;
				_last_seen_landmark_in_mcl_pose.ns = "acg";
				_last_seen_landmark_in_mcl_pose.id = 0;
				_last_seen_landmark_in_mcl_pose.scale.x = 0.5;
				_last_seen_landmark_in_mcl_pose.scale.y = 0.5;
				_last_seen_landmark_in_mcl_pose.color.b = 1.0f;
				_last_seen_landmark_in_mcl_pose.color.a = 1.0;


				_mcl_angles_markers.type = visualization_msgs::Marker::LINE_LIST;
				_mcl_angles_markers.header.frame_id = world_frame_id;
				_mcl_angles_markers.ns = "acg";
				_mcl_angles_markers.id = 0;
				_mcl_angles_markers.scale.x = 0.1;
				_mcl_angles_markers.scale.y = 0.1;
				_mcl_angles_markers.color.b = 1.0f;
				_mcl_angles_markers.color.a = 1.0;

				_robot_pose_angles_markers.type = visualization_msgs::Marker::LINE_LIST;
				_robot_pose_angles_markers.header.frame_id = world_frame_id;
				_robot_pose_angles_markers.ns = "acg";
				_robot_pose_angles_markers.id = 0;
				_robot_pose_angles_markers.scale.x = 0.1;
				_robot_pose_angles_markers.scale.y = 0.1;
				_robot_pose_angles_markers.color.b = 1.0f;
				_robot_pose_angles_markers.color.a = 1.0;

				_ndt_node_localization_markers.type = visualization_msgs::Marker::POINTS;
				_ndt_node_localization_markers.header.frame_id = world_frame_id;
				_ndt_node_localization_markers.ns = "acg";
				_ndt_node_localization_markers.id = 1;
				_ndt_node_localization_markers.scale.x = 0.2;
				_ndt_node_localization_markers.scale.y = 0.2;
				_ndt_node_localization_markers.color.r = 1.0f;
				_ndt_node_localization_markers.color.g = 1.0f;
				_ndt_node_localization_markers.color.a = 1.0;


				_ndt_cells.type = visualization_msgs::Marker::POINTS;
				_ndt_cells.header.frame_id = world_frame_id;
				_ndt_cells.ns = "acg";
				_ndt_cells.id = 0;
				_ndt_cells.scale.x = 0.15;
				_ndt_cells.scale.y = 0.15;
				_ndt_cells.color.g = 1.0f;
				_ndt_cells.color.a = 1.0;

				_ndt_cells_mcl.type = visualization_msgs::Marker::POINTS;
				_ndt_cells_mcl.header.frame_id = world_frame_id;
				_ndt_cells_mcl.ns = "acg";
				_ndt_cells_mcl.id = 0;
				_ndt_cells_mcl.scale.x = 0.15;
				_ndt_cells_mcl.scale.y = 0.15;
				_ndt_cells_mcl.color.b = 1.0f;
				_ndt_cells_mcl.color.g = 1.0f;
				_ndt_cells_mcl.color.a = 1.0;

				_ndt_cell_observations.type = visualization_msgs::Marker::LINE_LIST;
				_ndt_cell_observations.header.frame_id = world_frame_id;
				_ndt_cell_observations.ns = "acg";
				_ndt_cell_observations.id = 0;
				_ndt_cell_observations.scale.x = 0.1;
				_ndt_cell_observations.scale.y = 0.1;
				_ndt_cell_observations.color.r = 1.0f;
				_ndt_cell_observations.color.a = 1.0;

				_ndt_cell_associations.type = visualization_msgs::Marker::LINE_LIST;
				_ndt_cell_associations.header.frame_id = world_frame_id;
				_ndt_cell_associations.ns = "acg";
				_ndt_cell_associations.id = 0;
				_ndt_cell_associations.scale.x = 0.05;
				_ndt_cell_associations.scale.y = 0.05;
				_ndt_cell_associations.color.r = 1.0f;
				_ndt_cell_associations.color.b = 1.0f;
				_ndt_cell_associations.color.a = 1.0;


				_correct_robot_pose.type = visualization_msgs::Marker::POINTS;
				_correct_robot_pose.header.frame_id = world_frame_id;
				_correct_robot_pose.ns = "acg";
				_correct_robot_pose.id = 0;
				_correct_robot_pose.scale.x = 0.1;
				_correct_robot_pose.scale.y = 0.1;
				_correct_robot_pose.color.b = 1.0f;
				_correct_robot_pose.color.a = 1.0;

				_mcl_poses.type = visualization_msgs::Marker::POINTS;
				_mcl_poses.header.frame_id = world_frame_id;
				_mcl_poses.ns = "acg";
				_mcl_poses.id = 0;
				_mcl_poses.scale.x = 0.5;
				_mcl_poses.scale.y = 0.5;
				_mcl_poses.color.b = 0.5f;
				_mcl_poses.color.g = 0.5f;
				_mcl_poses.color.a = 1.0;


			}

			void drawLocalizations(const AutoCompleteGraphLocalization& acg);
			void drawPoseLocalizations(const AutoCompleteGraphLocalization& acg);
			void drawPriorObservations(const AutoCompleteGraphLocalization& acg);
			void drawCorrectRobotPoses(const AutoCompleteGraphLocalization& acg);

			void drawPrior(const AutoCompleteGraphLocalization& acg);
			void drawLocalizationLandmarks(const AutoCompleteGraphLocalization& acg);
			void drawMCL(const AutoCompleteGraphLocalization& acg);
			void drawMCLAngles(const AutoCompleteGraphLocalization &acg);
			void drawNDTCellObservations(const AutoCompleteGraphLocalization &acg);
			void drawNDTCellAssociations(const AutoCompleteGraphLocalization &acg);
			void drawNDTCells(const AutoCompleteGraphLocalization &acg);
			void drawNDTCellsMCL(const AutoCompleteGraphLocalization &acg);


			void updateRviz(const AutoCompleteGraphLocalization& acg){

//				std::cout << "What the fuck " << _nb_of_zone << " and " << acg.getRobotPoseLocalization().size() << std::endl;
//				exit(0);
				if(_nb_of_zone != acg.getRobotPoseLocalization().size()) {
					drawLocalizations(acg);
					drawMCLAngles(acg);
					drawPoseLocalizations(acg);
					drawPriorObservations(acg);
					drawLocalizationLandmarks(acg);
					drawCorrectRobotPoses(acg);
					drawMCL(acg);
					drawNDTCellsMCL(acg);

// 					initOccupancyGrid(*omap, 250, 250, 0.4, "/world");
					if(acg.getRobotPoseLocalization().size() > 0) {
						nav_msgs::OccupancyGrid omap_occ;
						int size_rl = acg.getRobotPoseLocalization().size();
						perception_oru::toOccupancyGrid(acg.getRobotPoseLocalization()[size_rl - 1]->getMap().get(), omap_occ, 0.1,
						                                "/world");
						_last_ndtmap2_occ.publish(omap_occ);
					}
//					_nb_of_zone = acg.getRobotNodes().size();
				}

				VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>::updateRviz(acg);

				drawNDTCellObservations(acg);
				drawNDTCellAssociations(acg);
				drawNDTCells(acg);
//
//
//					auto_complete_graph::ACGMaps mapmsg;
//					std::cout << "PUSH acg maps message" << std::endl;
//					AASS::acg::ACGToACGMapsMsg<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>(acg, mapmsg);
//					_acg_gdim.publish(mapmsg);
//
//					_nb_of_zone = acg.getRobotPoseLocalization().size();
//				}

			}


		};






		inline void VisuAutoCompleteGraphLocalization::drawLocalizations(const AutoCompleteGraphLocalization& acg)
		{
			_localization_edge_markers.header.stamp = ros::Time::now();
			auto edges = acg.getLocalizationEdges();
			if(edges.size() != _localization_edge_markers.points.size()){
				_localization_edge_markers.points.clear();
				auto it = edges.begin();
				for(it ; it != edges.end() ; ++it){
					for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){

						geometry_msgs::Point p;

						g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
						g2o::VertexPointXYACG* ptr2 = dynamic_cast<g2o::VertexPointXYACG*>((*ite2));

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

						_localization_edge_markers.points.push_back(p);
					}
				}
			}
			_localization_pub.publish(_localization_edge_markers);
		}


		inline void VisuAutoCompleteGraphLocalization::drawCorrectRobotPoses(const AutoCompleteGraphLocalization& acg)
		{
			_correct_robot_pose.header.stamp = ros::Time::now();
			auto poses = acg.getRobotPoseLocalization();
			if(poses.size() != _correct_robot_pose.points.size()){
			_correct_robot_pose.points.clear();
			for(auto pose : poses){
//					for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){

				geometry_msgs::Point p;
				Eigen::Affine3d pose_affine = pose->getPose();
				Eigen::Isometry2d pose_iso = Affine3d2Isometry2d(pose_affine);
				g2o::SE2 pose_se2(pose_iso);
				p.x = pose_se2.toVector()(0);
				p.y = pose_se2.toVector()(1);
				p.z = 0;
				_correct_robot_pose.points.push_back(p);
				}
			}

			_correct_robot_pose_pub.publish(_correct_robot_pose);
		}

		inline void VisuAutoCompleteGraphLocalization::drawLocalizationLandmarks(const AutoCompleteGraphLocalization& acg)
		{
			_last_seen_landmark_in_mcl_pose.header.stamp = ros::Time::now();
			_last_seen_landmark_in_mcl_pose.points.clear();

			std::cout << "There are " << acg.getLandmarkNodes().size() << " nodes " << std::endl;

			for(auto landmark : acg.getLandmarkNodes()){

				std::unordered_set <std::shared_ptr<AASS::acg::LocalizationPointer>> localization = landmark->getLocalization();

				for (auto loc: localization) {
					auto locali = acg.getRobotPoseLocalization();

					if(loc->vertex_mcl_pose == locali[ acg.getRobotPoseLocalization().size() -1 ] ){
						geometry_msgs::Point p;
						Eigen::Vector2d pose_landmark = loc->landmarkSeenByMCLInGlobalFrame().head(2);
						p.x = pose_landmark(0);
						p.y = pose_landmark(1);
						p.z = 0;
						_last_seen_landmark_in_mcl_pose.points.push_back(p);
					}

				}
					//The landmark pose should be in the mcl pose frame and not the robot mapping frame.

			}

			_last_landmark.publish(_last_seen_landmark_in_mcl_pose);
		}

		inline  void VisuAutoCompleteGraphLocalization::drawPoseLocalizations(const AutoCompleteGraphLocalization &acg) {

			_ndt_node_localization_markers.header.stamp = ros::Time::now();
			_ndt_node_localization_markers.points.clear();
			auto loc_vec = acg.getRobotPoseLocalization();
			for(auto loc : loc_vec) {

				geometry_msgs::Point p;
				g2o::VertexSE2RobotLocalization* ptr = dynamic_cast<g2o::VertexSE2RobotLocalization*>(loc);
				auto vertex = ptr->localizationInGlobalFrame();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = acg.getZElevation();
				_ndt_node_localization_markers.points.push_back(p);


			}
			_localization_pose_pub.publish(_ndt_node_localization_markers);
		}

		inline  void VisuAutoCompleteGraphLocalization::drawPriorObservations(const AutoCompleteGraphLocalization &acg) {


//			std::cout << "Prior observatrion print DRAWING " << _prior_observations.points.size() << std::endl;
//			int aaa;
//			std::cin >> aaa;

			_prior_observations.points.clear();
			_prior_observations.header.stamp = ros::Time::now();
			auto observations = acg.getPriorObservations();
			for(auto obs : observations) {

				bool from_mcl = false;
				for(auto vertex : obs->vertices() ){
					geometry_msgs::Point p;
					g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>(vertex);
					if(ptr != NULL){
						from_mcl = true;
					}
				}
				if(from_mcl == true){

					int old_size = _prior_observations.points.size();

					Eigen::Vector2d measurement = obs->measurement();
					Eigen::Vector3d measurement3d; measurement3d << measurement(0), measurement(1), 0;

					g2o::SE2 robot_frame;
					for(auto vertex : obs->vertices() ){
						g2o::VertexSE2RobotLocalization* ptr = dynamic_cast<g2o::VertexSE2RobotLocalization*>(vertex);
						if(ptr != NULL){
							robot_frame = ptr->estimate();
							auto vertex = robot_frame.toVector();
							//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
							geometry_msgs::Point p;
							p.x = vertex(0);
							p.y = vertex(1);
							p.z = acg.getZElevation();
							_prior_observations.points.push_back(p);
						}
					}

					Eigen::Vector3d pose_inglobal_frame;
					translateFromRobotFrameToGlobalFrame(measurement3d, robot_frame, pose_inglobal_frame);

					geometry_msgs::Point p2;
					p2.x = pose_inglobal_frame(0);
					p2.y = pose_inglobal_frame(1);
					p2.z = acg.getZElevation();
					_prior_observations.points.push_back(p2);

					assert(_prior_observations.points.size() == old_size + 2);


//					for(auto vertex : obs->vertices() ){
//						geometry_msgs::Point p;
//						g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>(vertex);
//						g2o::VertexSE2RobotLocalization* ptr_loc = dynamic_cast<g2o::VertexSE2RobotLocalization*>(vertex);
////						g2o::VertexSE2RobotPose* ptr_robot_pose = dynamic_cast<g2o::VertexSE2RobotPose*>(vertex);
//						if(ptr != NULL){
//							auto vertex = ptr->estimate();
//							//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
//							p.x = vertex(0);
//							p.y = vertex(1);
//							p.z = acg.getZElevation();
////							_prior_observations.points.push_back(p);
//
//							assert(ptr->landmarks.size() > 0);
//
//							bool empty = true;
//							if(_prior_observations.points.size() > 0){
//								std::cout << "PRIOR OBS " << p.x << " " << p.y << std::endl;
//								_prior_observations.points.push_back(p);
//								empty = false;
//							}
//
//							std::cout << "Landmarks " << ptr->landmarks.size() << std::endl;
//							for(auto land : ptr->landmarks){
//								geometry_msgs::Point p_land;
//								auto vertex_land = land->estimate();
//								//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
//								p_land.x = vertex_land(0);
//								p_land.y = vertex_land(1);
//								p_land.z = acg.getZElevation();
//								_prior_observations.points.push_back(p);
//								_prior_observations.points.push_back(p_land);
//								std::cout << "Between: " << vertex_land << " and the prior " << vertex << std::endl;
//							}
//
//							if(empty) {
//								std::cout << "PRIOR OBS " << p.x << " " << p.y << std::endl;
//								_prior_observations.points.push_back(p);
//							}
//
//						}
//						else if(ptr_loc != NULL){
//							auto vertex = ptr_loc->estimate().toVector();
//							//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
//							p.x = vertex(0);
//							p.y = vertex(1);
//							p.z = acg.getZElevation();
//							std::cout << "PRIOR OBS Loc " << p.x << " " << p.y << std::endl;
//							_prior_observations.points.push_back(p);
//						}
////						else if(ptr_robot_pose != NULL){
////							auto vertex = ptr_robot_pose->estimate().toVector();
////							//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
////							p.x = vertex(0);
////							p.y = vertex(1);
////							p.z = acg.getZElevation();
////							std::cout << "PRIOR OBS Robot pose " << p.x << " " << p.y << std::endl;
////							_prior_observations.points.push_back(p);
////						}
//						else{
//							throw std::runtime_error("Vertex type not found in visu of mcl observation edge");
//						}
//					}
				}
				else{
					ROS_DEBUG_STREAM("Not from mcl");
				}

			}
//			std::cout << "Prior observatrion print " << _prior_observations.points.size() << std::endl;
			_prior_observations_pub.publish(_prior_observations);
		}

		inline void VisuAutoCompleteGraphLocalization::drawMCL(const AutoCompleteGraphLocalization &acg){
			_mcl_poses.header.stamp = ros::Time::now();
			_mcl_poses.points.clear();
			for(auto robotpose : acg.getRobotPoseLocalization()){
				Eigen::Vector3d loc = robotpose->localizationInGlobalFrame();

				geometry_msgs::Point p;
				p.x = loc(0);
				p.y = loc(1);
				p.z = acg.getZElevation();
				_mcl_poses.points.push_back(p);
			}

			_mcl_localization.publish(_mcl_poses);
		}

		inline void VisuAutoCompleteGraphLocalization::drawMCLAngles(const AutoCompleteGraphLocalization &acg) {
			ROS_DEBUG_STREAM("Getting the angles");
			_mcl_angles_markers.header.stamp = ros::Time::now();
			auto localizations = acg.getRobotPoseLocalization();
			_mcl_angles_markers.points.clear();

			for(auto loc : localizations){

				geometry_msgs::Point p;
				// 				VertexLandmarkNDT* ptr = dynamic_cast<g2o::VertexPointXYACG*>((*it));
				auto vertex = loc->localizationInGlobalFrame();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = acg.getZElevation();
				_mcl_angles_markers.points.push_back(p);

				// 				std::cout << "getting the angle" << std::endl;

				double angle = vertex(2);
//				double anglew = (*it)->getAngleWidth(i);

				// 				std::cout << "angle " << angle<< std::endl;
				geometry_msgs::Point p2;
				p2.x = p.x + (2 * std::cos(angle));
				p2.y = p.y + (2 * std::sin(angle));
				p2.z = acg.getZElevation();
				_mcl_angles_markers.points.push_back(p2);

			}

			_mcl_angles_pub.publish(_mcl_angles_markers);

			auto robotpose = acg.getRobotNodes();
			_robot_pose_angles_markers.points.clear();

			for(auto robop : robotpose){

				geometry_msgs::Point p;
				// 				VertexLandmarkNDT* ptr = dynamic_cast<g2o::VertexPointXYACG*>((*it));
				auto vertex = robop->estimate().toVector();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = acg.getZElevation();
				_robot_pose_angles_markers.points.push_back(p);

				// 				std::cout << "getting the angle" << std::endl;

				double angle = vertex(2);
//				double anglew = (*it)->getAngleWidth(i);

				// 				std::cout << "angle " << angle<< std::endl;
				geometry_msgs::Point p2;
				p2.x = p.x + (2 * std::cos(angle));
				p2.y = p.y + (2 * std::sin(angle));
				p2.z = acg.getZElevation();
				_robot_pose_angles_markers.points.push_back(p2);

			}
//
			_robot_pose_angles_pub.publish(_robot_pose_angles_markers);
		}



		//Specialized this one
		template<>
		inline void AASS::acg::VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>::drawPrior(const AASS::acg::AutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>& acg)
		{
//			std::cout << "Drawing the prior" << std::endl;
			_prior_edge_markers.header.stamp = ros::Time::now();
			auto edges = acg.getPrior()->getEdges();
			if(edges.size() != _prior_edge_markers.points.size()){
				_prior_edge_markers.points.clear();

				auto it = edges.begin();
				for(it ; it != edges.end() ; ++it){
					for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
						geometry_msgs::Point p;
						g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>((*ite2));
						auto vertex = ptr->estimate();
						//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
						p.x = vertex(0);
						p.y = vertex(1);
						p.z = acg.getZElevation();
						_prior_edge_markers.points.push_back(p);
					}
				}

				auto prior_node = acg.getPrior()->getNodes();
				_prior_node_markers.points.clear();
				_angles_prior_markers.points.clear();
				_anglesw_prior_markers.points.clear();
				_angles_prior_markers.header.stamp = ros::Time::now();
				_anglesw_prior_markers.header.stamp = ros::Time::now();
				auto itt = prior_node.begin();
				for(itt ; itt != prior_node.end() ; ++itt){

					geometry_msgs::Point p;
					g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>((*itt));
					auto vertex = ptr->estimate();
					//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
					p.x = vertex(0);
					p.y = vertex(1);
					p.z = acg.getZElevation();
					_prior_node_markers.points.push_back(p);

// 				std::cout << "Drawing angles landmark" << std::endl;
					//NO PRIOR ANGLE YET HERE
					drawPriorAngles(acg, *ptr, vertex);

				}
			}else{
				ROS_DEBUG_STREAM("SAME SIZE PRIOR " );
			}
//			std::cout << "Edges size prior " << _prior_edge_markers.points.size() << std::endl;
			_marker_pub.publish(_prior_edge_markers);
			_prior_node_pub.publish(_prior_node_markers);
			_angles_prior_pub.publish(_angles_prior_markers);
			_anglesw_prior_pub.publish(_anglesw_prior_markers);

//			exit(0);
		}



		inline void VisuAutoCompleteGraphLocalization::drawNDTCellObservations(const AutoCompleteGraphLocalization &acg){
			_ndt_cell_observations.header.stamp = ros::Time::now();
			auto edges = acg.getNDTCellObservations();

//			if(edges.size() != _ndt_cell_observations.points.size()){

				_ndt_cell_observations.points.clear();

				for(auto edge : edges){


					for(auto ite2 = edge->vertices().begin(); ite2 != edge->vertices().end() ; ++ite2){

						geometry_msgs::Point p;

						g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
						g2o::VertexPointXYACG* ptr2 = dynamic_cast<g2o::VertexPointXYACG*>((*ite2));

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

						_ndt_cell_observations.points.push_back(p);
					}
				}
//			}
			_ndt_cell_observation_pub.publish(_ndt_cell_observations);
		}

		inline void VisuAutoCompleteGraphLocalization::drawNDTCellAssociations(const AutoCompleteGraphLocalization &acg){
			_ndt_cell_associations.header.stamp = ros::Time::now();
			auto edges = acg.getNDTCellAssociations();

//			if(edges.size() != _ndt_cell_observations.points.size()){

			_ndt_cell_associations.points.clear();

			for(auto edge : edges){

				g2o::VertexNDTCell* cell =  dynamic_cast<g2o::VertexNDTCell*>(edge->vertices()[0]);

				geometry_msgs::Point p_cell;
				p_cell.x = cell->estimate()(0);
				p_cell.y = cell->estimate()(1);
				p_cell.z = 0;

				auto wall = edge->getPriorWall();

				for(auto vert : wall->vertices()){
					g2o::VertexPointXYACG* ptr = dynamic_cast<g2o::VertexPointXYACG*>(vert);

					geometry_msgs::Point p;
					auto vertex = ptr->estimate();
					p.x = vertex(0);
					p.y = vertex(1);
					p.z = 0;

					_ndt_cell_associations.points.push_back(p);
					_ndt_cell_associations.points.push_back(p_cell);
				}
			}
//			}
			_ndt_cell_association_pub.publish(_ndt_cell_associations);
		}
		inline void VisuAutoCompleteGraphLocalization::drawNDTCells(const AutoCompleteGraphLocalization &acg){
			_ndt_cells.header.stamp = ros::Time::now();
			_ndt_cells.points.clear();
			auto node_ndtcell = acg.getNDTCells();
			for(auto cell : node_ndtcell) {

				geometry_msgs::Point p;
//				g2o::VertexSE2RobotLocalization* ptr = dynamic_cast<g2o::VertexSE2RobotLocalization*>(cell);
				auto vertex = cell->estimate();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = acg.getZElevation();
				_ndt_cells.points.push_back(p);

			}
			_ndt_cell_pub.publish(_ndt_cells);
		}

		inline void VisuAutoCompleteGraphLocalization::drawNDTCellsMCL(const AutoCompleteGraphLocalization &acg){
			_ndt_cells_mcl.header.stamp = ros::Time::now();
			_ndt_cells_mcl.points.clear();
//			auto node_ndtcell = acg.getNDTCellObservations();
			for(auto edge : acg.getNDTCellObservations()) {

				g2o::VertexSE2RobotLocalization* cell_robot;
				g2o::VertexNDTCell* cell_node;

				g2o::VertexSE2RobotLocalization* cell =  dynamic_cast<g2o::VertexSE2RobotLocalization*>(edge->vertices()[0]);
				if(cell != NULL){
					cell_robot = cell;
					cell_node = dynamic_cast<g2o::VertexNDTCell*>(edge->vertices()[1]);
					assert(cell_node != NULL);
				}
				else{
					cell_node = dynamic_cast<g2o::VertexNDTCell*>(edge->vertices()[0]);;
					cell_robot = dynamic_cast<g2o::VertexSE2RobotLocalization*>(edge->vertices()[1]);
					assert(cell_node != NULL);
					assert(cell_robot != NULL);
				}

				Eigen::Vector3d loc = cell_robot->localizationInGlobalFrame();
				Eigen::Vector3d loc_read = cell_robot->estimate().toVector();

				Eigen::Vector3d diff = loc - loc_read;
				g2o::SE2 diff_se2(diff);

				Eigen::Vector2d cell_pose_t = cell_node->estimate();
				Eigen::Vector3d cell_pose; cell_pose << cell_pose_t(0), cell_pose_t(1), loc_read(2);

				Eigen::Vector2d cell_trans = cell_pose_t - loc_read.head(2);
				g2o::SE2 loc_real_se2(loc_read);
				g2o::SE2 cell_pose_real(cell_pose);

				g2o::SE2 diff_se2_cell_loc = loc_real_se2.inverse() * cell_pose_real;

				Eigen::Vector3d cell_trans_3d; cell_trans_3d << cell_trans(0), cell_trans(1), loc(2);
				g2o::SE2 cell_trans_se2(cell_trans_3d);
				g2o::SE2 loc_se2(loc);
				g2o::SE2 new_cell_pose = loc_se2 * diff_se2_cell_loc;

//				g2o::SE2 cell_pose_se2(cell_pose);
//				cell_pose_se2 = cell_pose_se2 * diff_se2;

				geometry_msgs::Point p;
//				g2o::VertexSE2RobotLocalization* ptr = dynamic_cast<g2o::VertexSE2RobotLocalization*>(cell);
//				auto vertex = cell->estimate();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = new_cell_pose.toVector()(0);
				p.y = new_cell_pose.toVector()(1);
				p.z = acg.getZElevation();
				_ndt_cells_mcl.points.push_back(p);

			}
			_mcl_ndt_cell_pub.publish(_ndt_cells_mcl);
		}



	}
}
#endif