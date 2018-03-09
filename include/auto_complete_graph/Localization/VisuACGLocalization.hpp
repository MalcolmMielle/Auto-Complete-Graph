#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACGLOCALIZATION_09032018

#include "auto_complete_graph/VisuACG.hpp"
#include "auto_complete_graph/Localization/ACG_localization.hpp"

namespace AASS {

	namespace acg {


		class VisuAutoCompleteGraphLocalization : public VisuAutoCompleteGraph {

		protected:

			visualization_msgs::Marker _localization_edge_markers;
			ros::Publisher _localization_pub;

			AutoCompleteGraphLocalization *_acg;


		public:
			VisuAutoCompleteGraphLocalization(AutoCompleteGraphLocalization *acg, const ros::NodeHandle &nh)
					: VisuAutoCompleteGraph(acg, nh) {

				_acg = acg;

				_localization_pub = _nh.advertise<visualization_msgs::Marker>("localization_markers", 10);
				_localization_edge_markers.type = visualization_msgs::Marker::LINE_LIST;
				_localization_edge_markers.header.frame_id = "/world";
				_localization_edge_markers.ns = "acg";
				_localization_edge_markers.id = 0;
				_localization_edge_markers.scale.x = 0.2;
				_localization_edge_markers.scale.y = 0.2;
				_localization_edge_markers.color.b = 1.0f;
				_localization_edge_markers.color.a = 1.0;

			}

			void drawLocalizations();




			void updateRviz(){

				VisuAutoCompleteGraph::updateRviz();

				drawLocalizations();

			}


		};



		inline void VisuAutoCompleteGraphLocalization::drawLocalizations()
		{
			_localization_edge_markers.header.stamp = ros::Time::now();
			auto edges = _acg->getLocalizationEdges();
			if(edges.size() != _localization_edge_markers.points.size()){
				_localization_edge_markers.points.clear();
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

						_localization_edge_markers.points.push_back(p);
					}
				}
			}
			_localization_pub.publish(_localization_edge_markers);
		}








	}
}
#endif