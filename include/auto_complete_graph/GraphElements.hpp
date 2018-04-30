#ifndef AUTOCOMPLETEGRAPH_GRAPHELEMENT_30042018
#define AUTOCOMPLETEGRAPH_GRAPHELEMENT_30042018

#include <ctime>
#include <fstream>
#include <random>
#include <vector>
#include <set>
#include "g2o/types/slam2d/parameter_se2_offset.h"
//#include "ndt_feature/ndt_feature_graph.h"
#include "Eigen/Core"
//#include "bettergraph/PseudoGraph.hpp"
//#include "vodigrex/linefollower/SimpleNode.hpp"
//#include "ndt_feature_finder/ndt_corner.hpp"
#include "covariance.hpp"
//#include "conversion.hpp"
//#include "OptimizableAutoCompleteGraph.hpp"
#include "PriorLoaderInterface.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include "ndt_feature_finder/conversion.hpp"
//#include "utils.hpp"
//#include "VertexAndEdge/EdgeInterfaceMalcolm.hpp"
//#include "VertexAndEdge/EdgeOdometry.hpp"
//#include "VertexAndEdge/EdgeLandmark.hpp"
//#include "VertexAndEdge/EdgeLinkXY.hpp"
//#include "VertexAndEdge/VertexSE2RobotPose.hpp"


//TODO BUT LATER

namespace AASS {

	namespace acg {

		/**
		 * @brief The graph class containing all elements from the prior. Needed for the templated version of ACGLocalization :(.
		 */
		template<typename VERTEXTYPE, typename EDGETYPE>
		class GraphElements {

		protected:
			///@brief vector storing all node from the prior
			std::set<VERTEXTYPE*> _nodes;
			///@brief vector storing all edge between the prior nodes
			std::set<EDGETYPE*> _edges;

//			Eigen::Vector2d _priorNoise;
//			double _prior_rot;
			g2o::ParameterSE2Offset* _sensorOffset;

//			bool _use_user_prior_cov = false;

		public:

			GraphElements(const g2o::SE2& sensoffset) {
				// add the parameter representing the sensor offset ATTENTION was ist das ?
				_sensorOffset = new g2o::ParameterSE2Offset;
				_sensorOffset->setOffset(sensoffset);
				_sensorOffset->setId(0);
			}
//			GraphElements(const g2o::SE2& sensoffset){
//				// add the parameter representing the sensor offset ATTENTION was ist das ?
//				_sensorOffset = new g2o::ParameterSE2Offset;
//				_sensorOffset->setOffset(sensoffset);
//				_sensorOffset->setId(0);
//			};


			/** Accessor**/
			typename std::set<VERTEXTYPE*>& getNodes(){return _nodes;}
			const typename std::set<VERTEXTYPE*>& getNodes() const {return _nodes;}
			///@brief vector storing all edge between the prior nodes
			typename std::set<EDGETYPE*>& getEdges(){ return _edges;}
			const typename std::set<EDGETYPE*>& getEdges() const { return _edges;}

//			void setPriorNoise(double a, double b){_priorNoise << a, b;}
//			void setPriorRot(double r){_prior_rot = r;}
//
//			void useUserCovForPrior(bool u){_use_user_prior_cov = u;}
//			bool isUsingUserCovForPrior() const {return _use_user_prior_cov;}


			virtual VERTEXTYPE* addPose(const g2o::SE2& se2, const PriorAttr& priorAttr, int index) = 0;
			virtual VERTEXTYPE* addPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr, int index) = 0;
			virtual VERTEXTYPE* addPose(double x, double y, double theta, const PriorAttr& priorAttr, int index) = 0;

			virtual EDGETYPE* addEdge(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2) = 0;

			//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
			/**
			 * @brief Directly use the prior graph to init the prior part of the ACG
			 *
			 */
//			virtual int addPriorGraph(const PriorLoaderInterface::PriorGraph& graph, int first_index) = 0;
			///@remove the prior and all link edges
//			virtual void clearPrior() = 0;
//			virtual void checkNoRepeatingPriorEdge();

//			virtual pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(double resolution, double z_elevation, double varz) const  = 0;

			virtual void clear(){
				//It's a set so not needed
//				for(typename std::set<VERTEXTYPE*>::iterator it = getPriorNodes().begin() ; it != getPriorNodes().end() ; ++it){
//
////					for(auto it1 = it + 1 ; it1 != getPriorNodes().end() ;++it1){
////						assert(*it != *it1);
//////						++i;
////					}
//				}
				std::cout << "Clear the prior" << std::endl;
				_nodes.clear();
				_edges.clear();
				std::cout << "Prior cleared" << std::endl;
			}

			bool removeVertex(g2o::HyperGraph::Vertex* v1){
				//Prior
//				VERTEXTYPE* ptr = dynamic_cast<VERTEXTYPE*>(v1);

				if(v1 != NULL){
					std::cout <<"Found vertex" << std::endl;
					auto it = _nodes.find(v1);

					if(it != _nodes.end()) {
//					int index = findPriorNode(v1);
						_nodes.erase(it);
					}
					else{
						return false;
					}
				}
//				return NULL;
				return true;
			}


		};


//		template<typename VERTEXTYPE, typename EDGETYPE>
//		inline void AASS::acg::AutoCompleteGraphPrior<VERTEXTYPE, EDGETYPE>::checkNoRepeatingPriorEdge(){
//
//
//			for(auto edge : _edge_prior){
//				auto vertex = edge->vertices()[0];
//				auto vertex2 = edge->vertices()[1];
//				for(auto edge_second : _edge_prior){
//
//					if(edge != edge_second){
//						auto vertex_second = edge_second->vertices()[0];
//						auto vertex_second2 = edge_second->vertices()[1];
//
//						if(vertex_second == vertex && vertex_second2 == vertex2){
//							throw std::runtime_error("Same prior edge");
//						}
//						else if(vertex_second == vertex2 && vertex_second2 == vertex){
//							throw std::runtime_error("Same prior edge");
//						}
//
//					}
//
//				}
//
//			}


//			for(auto it_vertex = _nodes_prior.begin() ; it_vertex != _nodes_prior.end() ; ++it_vertex){
//				std::vector<std::pair<double, double> > out;
//				// 			std::cout << "edges " << std::endl;
//				auto edges = (*it_vertex)->edges();
//				// 			std::cout << "edges done " << std::endl;
//				std::vector<EDGETYPE*> edges_prior;
//
//				for ( auto ite = edges.begin(); ite != edges.end(); ++ite ){
//					// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeXYPrior*>(*ite) << std::endl;
////					EDGETYPE* ptr = dynamic_cast<EDGETYPE*>(*ite);
////					if(ptr != NULL){
//						//Make sure not pushed twice
//						for(auto ite2 = edges_prior.begin(); ite2 != edges_prior.end(); ++ite2 ){
//							assert(*ite != *ite2);
//						}
//						// 						std::cout << " pushed edges " << std::endl;
//						edges_prior.push_back(*ite);
//						// 						std::cout << "pushed edges done " << std::endl;
////					}
//				}
//				for(auto it = edges_prior.begin() ; it != edges_prior.end() ; ++it){
//					auto ite2 = it;
//					ite2++;
//					for( ; ite2 != edges_prior.end() ; ++ite2 ){
//						assert((*it)->getOrientation2D(**it_vertex) != (*ite2)->getOrientation2D(**it_vertex));
//					}
//				}
//			}
//			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
//				auto ite2 = it;
//				ite2++;
//				for(; ite2 != _edge_prior.end() ; ++ite2 ){
//					assert(it != ite2);
//				}
//			}
//		}
	}
}

#endif