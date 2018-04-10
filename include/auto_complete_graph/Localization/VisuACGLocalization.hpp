#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018

#include "auto_complete_graph/VisuACG.hpp"
#include "auto_complete_graph/Localization/ACG_localization.hpp"

namespace AASS {

	namespace acg {


		class VisuAutoCompleteGraphLocalization : public VisuAutoCompleteGraph {

		protected:

			visualization_msgs::Marker _localization_edge_markers;
			visualization_msgs::Marker _ndt_node_localization_markers;
			ros::Publisher _localization_pub;
			ros::Publisher _localization_pose_pub;

//			AutoCompleteGraphLocalization *_acg;


		public:
			VisuAutoCompleteGraphLocalization(const ros::NodeHandle &nh)
					: VisuAutoCompleteGraph(nh) {

//				_acg = acg;

				_localization_pub = _nh.advertise<visualization_msgs::Marker>("localization_markers", 10);
				_localization_pose_pub = _nh.advertise<visualization_msgs::Marker>("localization_pose_markers", 10);

				_localization_edge_markers.type = visualization_msgs::Marker::LINE_LIST;
				_localization_edge_markers.header.frame_id = "/world";
				_localization_edge_markers.ns = "acg";
				_localization_edge_markers.id = 0;
				_localization_edge_markers.scale.x = 0.1;
				_localization_edge_markers.scale.y = 0.1;
				_localization_edge_markers.color.b = 1.0f;
				_localization_edge_markers.color.a = 1.0;



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




			void updateRviz(const AutoCompleteGraphLocalization& acg){

				VisuAutoCompleteGraph::updateRviz(acg);
				drawLocalizations(acg);
				drawPoseLocalizations(acg);

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


	}
}
#endif