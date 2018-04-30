#ifndef AUTOCOMPLETEGRAPH_ACGPRIORSE2_13042018
#define AUTOCOMPLETEGRAPH_ACGPRIORSE2_13042018

#include "auto_complete_graph/ACGPrior.hpp"

#include "VertexAndEdge/EdgeSE2Prior.hpp"
#include "VertexAndEdge/VertexSE2Prior.hpp"


namespace AASS {

	namespace acg {



		/**
		 * @brief The graph class containing all elements from the prior. Needed for the templated version of ACGLocalization :(.
		 */
		class AutoCompleteGraphPriorSE2 : public AutoCompleteGraphPrior<g2o::VertexSE2Prior, g2o::EdgeSE2Prior_malcolm> {

		protected:
			///@brief vector storing all node from the prior
//			std::vector<g2o::VertexSE2Prior*> _nodes_prior;
			///@brief vector storing all edge between the prior nodes
//			std::vector<g2o::EdgeSE2Prior_malcolm*> _edge_prior;

		public:

			AutoCompleteGraphPriorSE2(const Eigen::Vector2d& pn, double rp, const g2o::SE2& sensoffset) : AutoCompleteGraphPrior(pn, rp, sensoffset){}
			AutoCompleteGraphPriorSE2(const g2o::SE2& sensoffset): AutoCompleteGraphPrior(sensoffset){}

			/** Accessor**/
//			std::vector<g2o::VertexSE2Prior*>& getPriorNodes(){return _nodes_prior;}
//			const std::vector<g2o::VertexSE2Prior*>& getPriorNodes() const {return _nodes_prior;}
			///@brief vector storing all edge between the prior nodes
//			std::vector<g2o::EdgeSE2Prior_malcolm*>& getPriorEdges(){ return _edge_prior;}
//			const std::vector<g2o::EdgeSE2Prior_malcolm*>& getPriorEdges() const { return _edge_prior;}


			virtual g2o::VertexSE2Prior* addPose(const g2o::SE2& se2, const PriorAttr& priorAttr, int index);
			virtual g2o::VertexSE2Prior* addPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr, int index);
			virtual g2o::VertexSE2Prior* addPose(double x, double y, double theta, const PriorAttr& priorAttr, int index);

			virtual g2o::EdgeSE2Prior_malcolm* addEdge(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);

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

			virtual pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(double resolution, double z_elevation, double varz) const ;


		};
	}
}

#endif