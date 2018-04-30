#ifndef AUTOCOMPLETEGRAPH_ACGPRIORXY_17042018
#define AUTOCOMPLETEGRAPH_ACGPRIORXY_17042018

#include "auto_complete_graph/ACGPrior.hpp"


#include "auto_complete_graph/VertexAndEdge/EdgeXYPrior.hpp"
#include "auto_complete_graph/VertexAndEdge/VertexXYPrior.hpp"


namespace AASS {

	namespace acg {



		/**
		 * @brief The graph class containing all elements from the prior. Needed for the templated version of ACGLocalization :(.
		 */
		class AutoCompleteGraphPriorXY : public AutoCompleteGraphPrior<g2o::VertexXYPrior, g2o::EdgeXYPriorACG> {

		protected:
			///@brief vector storing all node from the prior
//			std::vector<g2o::VertexXYPrior*> _nodes_prior;
			///@brief vector storing all edge between the prior nodes
//			std::vector<g2o::EdgeXYPrior*> _edge_prior;

		public:

			AutoCompleteGraphPriorXY(const Eigen::Vector2d& pn, double rp, const g2o::SE2& sensoffset) : AutoCompleteGraphPrior(pn, rp, sensoffset){}
			AutoCompleteGraphPriorXY(const g2o::SE2& sensoffset): AutoCompleteGraphPrior(sensoffset){}

			/** Accessor**/
//			std::vector<g2o::VertexXYPrior*>& getPriorNodes(){return _nodes_prior;}
//			const std::vector<g2o::VertexXYPrior*>& getPriorNodes() const {return _nodes_prior;}
			///@brief vector storing all edge between the prior nodes
//			std::vector<g2o::EdgeXYPrior*>& getPriorEdges(){ return _edge_prior;}
//			const std::vector<g2o::EdgeXYPrior*>& getPriorEdges() const { return _edge_prior;}


			virtual g2o::VertexXYPrior* addPose(const g2o::SE2& se2, const PriorAttr& priorAttr, int index);
			virtual g2o::VertexXYPrior* addPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr, int index);
			virtual g2o::VertexXYPrior* addPose(double x, double y, double theta, const PriorAttr& priorAttr, int index);

			virtual g2o::EdgeXYPriorACG* addEdge(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);

			//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
			/**
			 * @brief Directly use the prior graph to init the prior part of the ACG
			 *
			 */
			virtual int addPriorGraph(const PriorLoaderInterface::PriorGraph& graph, int first_index);


			///@remove the prior and all link edges
//			void clear();


//			void checkNoRepeatingPriorEdge();

			void updatePriorEdgeCovariance();

			void testNoNanInPrior(const std::string& before) const;

			pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(double resolution, double z_elevation, double varz)const ;


		};
	}
}

#endif