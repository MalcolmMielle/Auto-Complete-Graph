#ifndef AUTOCOMPLETEGRAPH_ACG_LOCALIZATION_07012018
#define AUTOCOMPLETEGRAPH_ACG_LOCALIZATION_07012018

#include "ACG.hpp"
#include "VertexAndEdge/EdgeLocalization.hpp"
//#include "graph_map/graph_map.h"
//#include "graph_map/GraphMapMsg.h"

namespace AASS {

namespace acg{	

    class AutoCompleteGraphlocalization : public AutoCompleteGraph{
        protected:
        
        
		std::vector<EdgeLocalization*> _edges_localization;
		AASS::acg::VertexLandmarkNDT* _vertex_reference_for_montecarlo;
        
        
        public:
        AutoCompleteGraph(const g2o::SE2& sensoffset, 
						const Eigen::Vector2d& tn, 
						double rn,
						const Eigen::Vector2d& ln,
						const Eigen::Vector2d& pn,
						double rp,
						const Eigen::Vector2d& linkn,
						ndt_feature::NDTFeatureGraph* ndt_graph
  					) : AutoCompletegraph(sensoffset, tn, rn, ln, pn, rp, linkn, ndt_graph){} 
  		
  		AutoCompleteGraph(const g2o::SE2& sensoffset, 
						  const Eigen::Vector2d& tn, 
						  double rn,
						  const Eigen::Vector2d& ln,
						  const Eigen::Vector2d& pn,
						  double rp,
						  const Eigen::Vector2d& linkn
					) : AutoCompletegraph(sensoffset, tn, rn, ln, pn, rp, linkn){}
		
		AutoCompleteGraph(const g2o::SE2& sensoffset, const std::string& load_file) : AutoCompletegraph(sensoffset, load_file){}
        
        
		std::vector<AASS::acg::EdgeLocalization*>& getLocalizationEdges(){return _edges_localization;}
		const std::vector<AASS::acg::EdgeLocalization*>& getLocalizationEdges() const {return _edges_localization;}
		
		
		/** FUNCTION TO ADD THE EGDES **/
//		EdgeLocalization* addLocalization(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, const Eigen::Matrix3d& information_tmp);
//		EdgeLocalization* addLocalization(const g2o::SE2& observ, int from_id, const Eigen::Matrix3d& information);
//		EdgeLocalization* addLocalization(double x, double y, double theta, int from_id, const Eigen::Matrix3d& information);
//		EdgeLocalization* addLocalization(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1);
//		EdgeLocalization* addLocalization(const g2o::SE2& observ, int from_id);
//		EdgeLocalization* addLocalization(double x, double y, double theta, int from_id);
		
		/** Others **/
		/**@brief set the vertex in the prior that is going to be the reference point for the localization. return NULL if failed, otherwise return the pointer to the vertex that was chosen
		**/
//		AASS::acg::VertexSE2Prior* setPriorReference();
		
		/**
		 * @brief Incrementally update the NDTGraph UPDATED TO THE NEW VERSION :)
		 * Only the new nodes are added to the graph. If the g2o graph has 4 nodes, only nodes 5 to last node of the NDT graph are added to it.
		 * Add NDT-corners and Robot poses.
		 */
//		void updateNDTGraph(graph_map::GraphMapMsg& ndt_graph, bool noise_flag = false, double deviation = 0.5);



    };

}


#endif
