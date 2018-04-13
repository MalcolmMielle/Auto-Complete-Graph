#ifndef AUTOCOMPLETEGRAPH_ACGPRIOR_13042018
#define AUTOCOMPLETEGRAPH_ACGPRIOR_13042018

#include <ctime>
#include <fstream>
#include <random>
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "ndt_feature/ndt_feature_graph.h"
#include "Eigen/Core"
#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
#include "ndt_feature_finder/ndt_corner.hpp"
#include "covariance.hpp"
#include "conversion.hpp"
#include "OptimizableAutoCompleteGraph.hpp"
#include "PriorLoaderInterface.hpp"
#include "ndt_feature_finder/conversion.hpp"
#include "utils.hpp"
#include "VertexAndEdge/EdgeInterfaceMalcolm.hpp"
#include "VertexAndEdge/EdgeOdometry.hpp"
#include "VertexAndEdge/EdgeLandmark.hpp"
#include "VertexAndEdge/EdgeLinkXY.hpp"
#include "VertexAndEdge/EdgeSE2Prior.hpp"
#include "VertexAndEdge/VertexLandmarkNDT.hpp"
#include "VertexAndEdge/VertexSE2Prior.hpp"
#include "VertexAndEdge/VertexSE2RobotPose.hpp"


namespace AASS {

	namespace acg {

		/**
		 * @brief The graph class containing all elements from the prior. Needed for the templated version of ACGLocalization :(.
		 */
		template<typename VERTEXTYPE, typename EDGETYPE>
		class AutoCompleteGraphPrior {

		protected:
			///@brief vector storing all node from the prior
			std::set<VERTEXTYPE*> _nodes_prior;
			///@brief vector storing all edge between the prior nodes
			std::set<EDGETYPE*> _edge_prior;

			Eigen::Vector2d _priorNoise;
			double _prior_rot;
			g2o::ParameterSE2Offset* _sensorOffset;

		public:

			AutoCompleteGraphPrior(const Eigen::Vector2d& pn, double rp, const g2o::SE2& sensoffset) : _priorNoise(pn), _prior_rot(rp){
				// add the parameter representing the sensor offset ATTENTION was ist das ?
				_sensorOffset = new g2o::ParameterSE2Offset;
				_sensorOffset->setOffset(sensoffset);
				_sensorOffset->setId(0);
			}
			AutoCompleteGraphPrior(const g2o::SE2& sensoffset){
				// add the parameter representing the sensor offset ATTENTION was ist das ?
				_sensorOffset = new g2o::ParameterSE2Offset;
				_sensorOffset->setOffset(sensoffset);
				_sensorOffset->setId(0);
			};


			/** Accessor**/
			typename std::set<VERTEXTYPE*>& getPriorNodes(){return _nodes_prior;}
			const typename std::set<VERTEXTYPE*>& getPriorNodes() const {return _nodes_prior;}
			///@brief vector storing all edge between the prior nodes
			typename std::set<EDGETYPE*>& getPriorEdges(){ return _edge_prior;}
			const typename std::set<EDGETYPE*>& getPriorEdges() const { return _edge_prior;}

			void setPriorNoise(double a, double b){_priorNoise << a, b;}
			void setPriorRot(double r){_prior_rot = r;}


			virtual VERTEXTYPE* addPriorLandmarkPose(const g2o::SE2& se2, const PriorAttr& priorAttr, int index) = 0;
			virtual VERTEXTYPE* addPriorLandmarkPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr, int index) = 0;
			virtual VERTEXTYPE* addPriorLandmarkPose(double x, double y, double theta, const PriorAttr& priorAttr, int index) = 0;

			virtual EDGETYPE* addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2) = 0;

			//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
			/**
			 * @brief Directly use the prior graph to init the prior part of the ACG
			 *
			 */
			virtual void addPriorGraph(const PriorLoaderInterface::PriorGraph& graph) = 0;
			///@remove the prior and all link edges
//			virtual void clearPrior() = 0;
			virtual void checkNoRepeatingPriorEdge() = 0;

			virtual void clear(){
				_nodes_prior.clear();
				_edge_prior.clear();
			}

			g2o::HyperGraph::Vertex* removeVertex(g2o::HyperGraph::Vertex* v1){
				//Prior
				VERTEXTYPE* ptr = dynamic_cast<VERTEXTYPE*>(v1);

				if(ptr != NULL){
					std::cout <<"Found vertex" << std::endl;
					auto it = _nodes_prior.find(ptr);
//					int index = findPriorNode(v1);
					_nodes_prior.erase(it);
				}
				return ptr;
			}


		};
	}
}

#endif