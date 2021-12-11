#ifndef AUTOCOMPLETEGRAPH_ACGPRIORXY_17042018
#define AUTOCOMPLETEGRAPH_ACGPRIORXY_17042018

#include "auto_complete_graph/ACGPrior.hpp"

#include "auto_complete_graph/VertexAndEdge/EdgeXYPrior.hpp"
#include "auto_complete_graph/VertexAndEdge/VertexXYPrior.hpp"

namespace AASS
{

	namespace acg
	{

		/**
		 * @brief The graph class containing all elements from the prior. Needed for the templated version of ACGLocalization :(.
		 */
		class AutoCompleteGraphPriorXY : public AutoCompleteGraphPrior<g2o::VertexXYPrior, g2o::EdgeXYPriorACG>
		{

		public:
			AutoCompleteGraphPriorXY(const Eigen::Vector2d &pn, double rp, const g2o::SE2 &sensoffset) : AutoCompleteGraphPrior(pn, rp, sensoffset) {}
			AutoCompleteGraphPriorXY(const g2o::SE2 &sensoffset) : AutoCompleteGraphPrior(sensoffset) {}

			virtual g2o::VertexXYPrior *addPose(const g2o::SE2 &se2, const PriorAttr &priorAttr, int index);
			virtual g2o::VertexXYPrior *addPose(const Eigen::Vector3d &lan, const PriorAttr &priorAttr, int index);
			virtual g2o::VertexXYPrior *addPose(double x, double y, double theta, const PriorAttr &priorAttr, int index);

			virtual g2o::EdgeXYPriorACG *addEdge(const g2o::SE2 &se2, g2o::HyperGraph::Vertex *v1, g2o::HyperGraph::Vertex *v2);

			//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
			/**
			 * @brief Directly use the prior graph to init the prior part of the ACG
			 *
			 */
			virtual int addPriorGraph(const PriorLoaderInterface::PriorGraph &graph, int first_index);

			void updatePriorEdgeCovariance();

			void testNoNanInPrior(const std::string &before) const;

			pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(double resolution, double z_elevation, double varz) const;
		};
	}
}

#endif