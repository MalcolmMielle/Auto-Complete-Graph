#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018

#include "auto_complete_graph/VisuACG.hpp"
#include "auto_complete_graph/Localization/ACG_localization.hpp"

namespace AASS {

	namespace acg {


		class VisuAutoCompleteGraphLocalization : public VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG> {

		protected:

			visualization_msgs::Marker _localization_edge_markers;
			visualization_msgs::Marker _prior_observations;
			visualization_msgs::Marker _ndt_node_localization_markers;
			visualization_msgs::Marker _mcl_angles_markers;
			ros::Publisher _localization_pub;
			ros::Publisher _localization_pose_pub;
			ros::Publisher _prior_observations_pub;
			ros::Publisher _mcl_angles_pub;

//			AutoCompleteGraphLocalization *_acg;


		public:
			VisuAutoCompleteGraphLocalization(const ros::NodeHandle &nh)
					: VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>(nh) {

//				_acg = acg;

				_localization_pub = _nh.advertise<visualization_msgs::Marker>("localization_markers", 10);
				_localization_pose_pub = _nh.advertise<visualization_msgs::Marker>("localization_pose_markers", 10);
				_prior_observations_pub = _nh.advertise<visualization_msgs::Marker>("prior_observations_markers", 10);
				_mcl_angles_pub = _nh.advertise<visualization_msgs::Marker>("mcl_angles_markers", 10);

				_localization_edge_markers.type = visualization_msgs::Marker::LINE_LIST;
				_localization_edge_markers.header.frame_id = "/world";
				_localization_edge_markers.ns = "acg";
				_localization_edge_markers.id = 0;
				_localization_edge_markers.scale.x = 0.1;
				_localization_edge_markers.scale.y = 0.1;
				_localization_edge_markers.color.b = 1.0f;
				_localization_edge_markers.color.a = 1.0;


				_prior_observations.type = visualization_msgs::Marker::LINE_LIST;
				_prior_observations.header.frame_id = "/world";
				_prior_observations.ns = "acg";
				_prior_observations.id = 0;
				_prior_observations.scale.x = 0.1;
				_prior_observations.scale.y = 0.1;
				_prior_observations.color.b = 1.0f;
				_prior_observations.color.a = 1.0;


				_mcl_angles_markers.type = visualization_msgs::Marker::LINE_LIST;
				_mcl_angles_markers.header.frame_id = "/world";
				_mcl_angles_markers.ns = "acg";
				_mcl_angles_markers.id = 0;
				_mcl_angles_markers.scale.x = 0.1;
				_mcl_angles_markers.scale.y = 0.1;
				_mcl_angles_markers.color.b = 1.0f;
				_mcl_angles_markers.color.a = 1.0;



				_ndt_node_localization_markers.type = visualization_msgs::Marker::POINTS;
				_ndt_node_localization_markers.header.frame_id = "/world";
				_ndt_node_localization_markers.ns = "acg";
				_ndt_node_localization_markers.id = 1;
				_ndt_node_localization_markers.scale.x = 0.2;
				_ndt_node_localization_markers.scale.y = 0.2;
				_ndt_node_localization_markers.color.r = 1.0f;
				_ndt_node_localization_markers.color.g = 1.0f;
				_ndt_node_localization_markers.color.a = 1.0;

			}

			void drawLocalizations(const AutoCompleteGraphLocalization& acg);
			void drawPoseLocalizations(const AutoCompleteGraphLocalization& acg);
			void drawPriorObservations(const AutoCompleteGraphLocalization& acg);

			void drawPrior(const AutoCompleteGraphLocalization& acg);

			void drawMCLAngles(const AutoCompleteGraphLocalization &acg);


			void updateRviz(const AutoCompleteGraphLocalization& acg){

				if(_nb_of_zone != acg.getRobotNodes().size()) {
					drawLocalizations(acg);
					drawMCLAngles(acg);
					drawPoseLocalizations(acg);
					drawPriorObservations(acg);
//					_nb_of_zone = acg.getRobotNodes().size();
				}
				VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>::updateRviz(acg);

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

		inline  void VisuAutoCompleteGraphLocalization::drawPoseLocalizations(const AutoCompleteGraphLocalization &acg) {

			_ndt_node_localization_markers.header.stamp = ros::Time::now();
			_ndt_node_localization_markers.points.clear();
			auto loc_vec = acg.getRobotPoseLocalization();
			for(auto loc : loc_vec) {

				geometry_msgs::Point p;
				g2o::VertexSE2RobotLocalization* ptr = dynamic_cast<g2o::VertexSE2RobotLocalization*>(loc);
				auto vertex = ptr->estimate().toVector();
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = acg.getZElevation();
				_ndt_node_localization_markers.points.push_back(p);


			}
			_localization_pose_pub.publish(_ndt_node_localization_markers);
		}

		inline  void VisuAutoCompleteGraphLocalization::drawPriorObservations(const AutoCompleteGraphLocalization &acg) {

			_prior_observations.points.clear();
			_prior_observations.header.stamp = ros::Time::now();
			auto observations = acg.getLandmarkEdges();
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
					for(auto vertex : obs->vertices() ){
						geometry_msgs::Point p;
						g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>(vertex);
						g2o::VertexSE2RobotLocalization* ptr_loc = dynamic_cast<g2o::VertexSE2RobotLocalization*>(vertex);
						if(ptr != NULL){
							auto vertex = ptr->estimate();
							//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
							p.x = vertex(0);
							p.y = vertex(1);
							p.z = acg.getZElevation();
//							_prior_observations.points.push_back(p);

							assert(ptr->landmarks.size() > 0);

							bool empty = true;
							if(_prior_observations.points.size() > 0){
								std::cout << "PRIOR OBS " << p.x << " " << p.y << std::endl;
								_prior_observations.points.push_back(p);
								empty = false;
							}

							std::cout << "Landmarks " << ptr->landmarks.size() << std::endl;
							for(auto land : ptr->landmarks){
								geometry_msgs::Point p_land;
								auto vertex_land = land->estimate();
								//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
								p_land.x = vertex_land(0);
								p_land.y = vertex_land(1);
								p_land.z = acg.getZElevation();
								_prior_observations.points.push_back(p);
								_prior_observations.points.push_back(p_land);
								std::cout << "Between: " << vertex_land << " and the prior " << vertex << std::endl;
							}

							if(empty) {
								std::cout << "PRIOR OBS " << p.x << " " << p.y << std::endl;
								_prior_observations.points.push_back(p);
							}

						}
						else if(ptr_loc != NULL){
							auto vertex = ptr_loc->estimate().toVector();
							//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
							p.x = vertex(0);
							p.y = vertex(1);
							p.z = acg.getZElevation();
							std::cout << "PRIOR OBS Loc " << p.x << " " << p.y << std::endl;
							_prior_observations.points.push_back(p);
						}
						else{
							throw std::runtime_error("Vertex type not found in visu of mcl observation edge");
						}
					}
				}
				else{
					std::cout << "Not from mcl" << std::endl;
				}

			}
			_prior_observations_pub.publish(_prior_observations);
		}

		inline void VisuAutoCompleteGraphLocalization::drawMCLAngles(const AutoCompleteGraphLocalization &acg) {
			std::cout << "Getting the angles" << std::endl;
			_mcl_angles_markers.header.stamp = ros::Time::now();
			auto localizations = acg.getRobotPoseLocalization();
			_mcl_angles_markers.points.clear();

			for(auto loc : localizations){

				geometry_msgs::Point p;
				// 				VertexLandmarkNDT* ptr = dynamic_cast<g2o::VertexPointXYACG*>((*it));
				auto vertex = loc->estimate().toVector();
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

			auto robotpose = acg.getRobotNodes();
			_mcl_angles_markers.points.clear();

			for(auto robop : robotpose){

				geometry_msgs::Point p;
				// 				VertexLandmarkNDT* ptr = dynamic_cast<g2o::VertexPointXYACG*>((*it));
				auto vertex = robop->estimate().toVector();
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
//
			_mcl_angles_pub.publish(_mcl_angles_markers);
		}



		//Specialized this one
		template<>
		inline void AASS::acg::VisuAutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>::drawPrior(const AASS::acg::AutoCompleteGraphBase<AutoCompleteGraphPriorXY, g2o::VertexXYPrior, g2o::EdgeXYPriorACG>& acg)
		{
			_prior_edge_markers.header.stamp = ros::Time::now();
			auto edges = acg.getPrior()->getPriorEdges();
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

				auto prior_node = acg.getPrior()->getPriorNodes();
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
//					drawPriorAngles(acg, *ptr);

				}
			}
			_marker_pub.publish(_prior_edge_markers);
			_prior_node_pub.publish(_prior_node_markers);
			_angles_prior_pub.publish(_angles_prior_markers);
			_anglesw_prior_pub.publish(_anglesw_prior_markers);
		}



	}
}
#endif