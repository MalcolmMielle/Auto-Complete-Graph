#ifndef AUTOCOMPLETEGRAPH_ACG_15092016
#define AUTOCOMPLETEGRAPH_ACG_15092016

#include <ctime>

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_se2_link.h"
#include "g2o/types/slam2d/edge_landmark_se2.h"
#include "g2o/types/slam2d/edge_link_xy.h"
#include "g2o/types/slam2d/vertex_se2_prior.h"
// #include "types_tutorial_slam2d.h"



// #include "g2o/core/sparse_optimizer.h"
// #include "g2o/core/block_solver.h"
// #include "g2o/core/factory.h"
// #include "g2o/core/optimization_algorithm_factory.h"
// #include "g2o/core/optimization_algorithm_gauss_newton.h"
// #include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "ndt_feature/ndt_feature_graph.h"
// #include "ndt_feature/utils.h"

#include "Eigen/Core"

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"

#include "das/AssociationInterface.hpp"
#include "das/NDTCorner.hpp"
#include "covariance.hpp"
#include "conversion.hpp"
#include "OptimizableAutoCompleteGraph.hpp"

namespace AASS {

namespace acg{	
	
	/**
	 * @brief The graph class containing all elemnts from each map and the graph used in the g2o optimisation.
	 * ATTENTION : _transNoise is not used anymore when update the NDT-graph for we used the registration. I should remove it but for now I'm leaving it only not to break everything.
	 */
	class AutoCompleteGraph{
		
		
private:
		int _previous_number_of_node_in_ndtgraph;
		
		//Minimum distance from a prior corner to a NDT corner. THe distance is given in meter and is fixed at 2m in the constuctor
		double _min_distance_for_link_in_meter;
		
		/**
		 * @brief : used in a function to update the NDTGraph
		 * */
		class NDTCornerGraphElement{
			public:
				cv::Point2f point;
			protected:
				//TODO : change it to a set
				std::vector<int> nodes_linked;
				std::vector<g2o::VertexSE2*> nodes_linked_ptr;
				std::vector<g2o::Vector2D> observations;
				
			public:
				NDTCornerGraphElement(float x, float y) : point(x, y){};
				NDTCornerGraphElement(const cv::Point2f& p) : point(p){};
				
				void addAllObserv(int i, g2o::VertexSE2* ptr, g2o::Vector2D obs){
					nodes_linked.push_back(i);
					observations.push_back(obs);
					nodes_linked_ptr.push_back(ptr);
				}
				
				size_t size(){
					assert(nodes_linked.size() == nodes_linked_ptr.size());
					assert(nodes_linked.size() == observations.size());
					return nodes_linked.size();
				}
				
		// 		std::vector<int>& getNodeLinked(){return nodes_linked;}
				const std::vector<int>& getNodeLinked() const {return nodes_linked;}
				const std::vector<g2o::VertexSE2*>& getNodeLinkedPtr() const {return nodes_linked_ptr;}
				const std::vector<g2o::Vector2D>& getObservations() const {return observations;}
				
		// 		void push_back(int i){nodes_linked.push_back(i);}
				
		// 		void addNode(int i){nodes_linked.push_back(i);}
		// 		void addObservation(const g2o::Vector2D& obs){ observations.push_back(obs);}
				
				void fuse(const NDTCornerGraphElement& cor){
					for(size_t i = 0 ; i < cor.getNodeLinked().size() ; ++i){
						bool seen = false;
						for(size_t j = 0 ; j < nodes_linked.size() ; ++j){
							if(cor.getNodeLinked()[i] == nodes_linked[j]){
								seen = true;
							}
						}
						if(seen == false){
							nodes_linked.push_back(cor.getNodeLinked()[i]);
							observations.push_back(cor.getObservations()[i]);
							nodes_linked_ptr.push_back(cor.getNodeLinkedPtr()[i]);
						}
					}
				}
				void print() const {std::cout << point << " nodes : ";
					
					for(size_t i = 0 ; i < nodes_linked.size()  ; ++i){
						std::cout << nodes_linked[i] << " " ;
					}
					
				}
				
			};

		/**
		* @brief A class that old a NDT node pointer for the g2o graph, the associated NDTMap and the original affine transform (the one received when the NDTGraph was received)
		*/
		class NDTNodeAndMap{
			g2o::VertexSE2* _node;
			lslgeneric::NDTMap* _map;
			Eigen::Affine3d _T;
		public:
			NDTNodeAndMap(g2o::VertexSE2* node, lslgeneric::NDTMap* map, const Eigen::Affine3d& T) : _node(node), _map(map), _T(T){};
			
	// 		~NDTNodeAndMap(){delete _map;}
			
			g2o::VertexSE2* getNode(){return _node;}
			void setNode(g2o::VertexSE2* node){_node = node;}
			lslgeneric::NDTMap* getMap(){return _map;}
			void setMap(lslgeneric::NDTMap* map){_map = map;}
			Eigen::Affine3d getPose(){return _T;}
			const Eigen::Affine3d& getPose() const {return _T;}
			
		};
		
		//ATTENTION Already useless
		class EdgePriorAndInitialValue{
		protected:
			g2o::EdgeSE2Prior_malcolm* _edge;
			g2o::SE2 _original_value;
			
		public:
			EdgePriorAndInitialValue(g2o::EdgeSE2Prior_malcolm* ed, const g2o::SE2& orig_val) : _edge(ed), _original_value(orig_val){}
			
			g2o::EdgeSE2Prior_malcolm* getEdge(){return _edge;}
			g2o::SE2 getOriginalValue(){return _original_value;}
		};
		
		
		
	protected:
		
		//Needed system values
		Eigen::Vector2d _transNoise;
		double _rotNoise;
		Eigen::Vector2d _landmarkNoise;
		Eigen::Vector2d _priorNoise;
		double _prior_rot;
		Eigen::Vector2d _linkNoise;
		g2o::ParameterSE2Offset* _sensorOffset;
		g2o::SE2 _sensorOffsetTransf;
		
		///@brief vector storing all node from the prior 
		std::vector<g2o::VertexSE2Prior*> _nodes_prior;
		///@brief vector storing all node from the landarks 
		std::vector<g2o::VertexPointXY*> _nodes_landmark;
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<NDTNodeAndMap> _nodes_ndt;
		///@brief vector storing all linking edges
		std::vector<g2o::EdgeLinkXY_malcolm*> _edge_link;
		///@brief vector storing all edges between a landmark and the robot
		std::vector<g2o::EdgeSE2PointXY*> _edge_landmark;
		///@brief vector storing all edge between the prior nodes
		std::vector<EdgePriorAndInitialValue> _edge_prior;
		///@brief vector storing the odometry
		std::vector<g2o::EdgeSE2*> _edge_odometry;
		
		///@brief the main dish : the graph
// 		g2o::OptimizableGraph _optimizable_graph;
		AASS::acg::OptimizableAutoCompleteGraph _optimizable_graph;
		
		//ATTENTION : I should avoid that if I want to run both thread at the same time since no copy is made. I should instead copy it
		ndt_feature::NDTFeatureGraph* _ndt_graph;
		
// 		std::vector < NDTCornerGraphElement > _ndt_corners;
		
		
	
	public:
		
		AutoCompleteGraph(const g2o::SE2& sensoffset, 
						const Eigen::Vector2d& tn, 
						double rn,
						const Eigen::Vector2d& ln,
						const Eigen::Vector2d& pn,
						double rp,
						const Eigen::Vector2d& linkn,
						ndt_feature::NDTFeatureGraph* ndt_graph
  					) : _sensorOffsetTransf(sensoffset), _transNoise(tn), _rotNoise(rn), _landmarkNoise(ln), _priorNoise(pn), _prior_rot(rp), _linkNoise(linkn), _previous_number_of_node_in_ndtgraph(0), _min_distance_for_link_in_meter(2), _optimizable_graph(sensoffset), _ndt_graph(ndt_graph){
						// add the parameter representing the sensor offset ATTENTION was ist das ?
						_sensorOffset = new g2o::ParameterSE2Offset;
						_sensorOffset->setOffset(_sensorOffsetTransf);
						_sensorOffset->setId(0);
					}
		
		AutoCompleteGraph(const g2o::SE2& sensoffset, 
						const Eigen::Vector2d& tn, 
						double rn,
						const Eigen::Vector2d& ln,
						const Eigen::Vector2d& pn,
						double rp,
						const Eigen::Vector2d& linkn
					) : _sensorOffsetTransf(sensoffset), _transNoise(tn), _rotNoise(rn), _landmarkNoise(ln), _priorNoise(pn), _prior_rot(rp), _linkNoise(linkn), _previous_number_of_node_in_ndtgraph(0), _min_distance_for_link_in_meter(2), _optimizable_graph(sensoffset){
						// add the parameter representing the sensor offset ATTENTION was ist das ?
						_sensorOffset = new g2o::ParameterSE2Offset;
						_sensorOffset->setOffset(_sensorOffsetTransf);
						_sensorOffset->setId(0);
						_ndt_graph = NULL;
						
					}
		~AutoCompleteGraph(){
			delete _sensorOffset;
			//The _optimizable_graph already delete the vertices in the destructor
			
// 			cleanup pointers in NODE
			
			for(size_t i = 0 ; i < _nodes_ndt.size() ; ++i){
				delete _nodes_ndt[i].getMap();
			}
			
		}
		
		/** Accessor**/
		std::vector<g2o::VertexSE2Prior*>& getPriorNodes(){return _nodes_prior;}
		const std::vector<g2o::VertexSE2Prior*>& getPriorNodes() const {return _nodes_prior;}
		///@brief vector storing all node from the prior 
		std::vector<g2o::VertexPointXY*>& getLandmarkNodes(){return _nodes_landmark;}
		const std::vector<g2o::VertexPointXY*>& getLandmarkNodes() const {return _nodes_landmark;}
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<NDTNodeAndMap>& getRobotNodes(){return _nodes_ndt;}
		const std::vector<NDTNodeAndMap>& getRobotNodes() const {return _nodes_ndt;}
		///@brief vector storing all linking edges
		std::vector<g2o::EdgeLinkXY_malcolm*>& getLinkEdges(){return _edge_link;}
		const std::vector<g2o::EdgeLinkXY_malcolm*>& getLinkEdges() const {return _edge_link;}
		///@brief vector storing all edges between a landmark and the robot
		std::vector<g2o::EdgeSE2PointXY*>& getLandmarkEdges(){return _edge_landmark;}
		const std::vector<g2o::EdgeSE2PointXY*>& getLandmarkEdges() const {return _edge_landmark;}
		///@brief vector storing all edge between the prior nodes
		std::vector<EdgePriorAndInitialValue>& getPriorEdges(){ return _edge_prior;}
		const std::vector<EdgePriorAndInitialValue>& getPriorEdges() const { return _edge_prior;}
		///@brief vector storing the odometry
		std::vector<g2o::EdgeSE2*>& getOdometryEdges(){return _edge_odometry;}
		const std::vector<g2o::EdgeSE2*>& getOdometryEdges() const {return _edge_odometry;}
		
		void setMinDistanceForLinksInMeters(double inpu){_min_distance_for_link_in_meter = inpu;}
		double getMinDistanceForLinksInMeters(){return _min_distance_for_link_in_meter;}
		
		bool save(const std::string& file_outt){
			_optimizable_graph.save(file_outt.c_str());
			std::cout << "saved to " << file_outt << "\n";
		}
		
		///@brief the main dish : the graph
		g2o::OptimizableGraph& getGraph(){return _optimizable_graph;}
		const g2o::OptimizableGraph& getGraph() const {return _optimizable_graph;}
		
		/***FUNCTIONS TO ADD THE NODES***/
		g2o::VertexSE2* addRobotPose(const g2o::SE2& se2, const Eigen::Affine3d& affine, lslgeneric::NDTMap* map);
		g2o::VertexSE2* addRobotPose(const Eigen::Vector3d& rob, const Eigen::Affine3d& affine, lslgeneric::NDTMap* map);
		g2o::VertexSE2* addRobotPose(double x, double y, double theta, const Eigen::Affine3d& affine, lslgeneric::NDTMap* map);
		
		g2o::VertexPointXY* addLandmarkPose(const g2o::Vector2D& pos, int strength = 1);
		g2o::VertexPointXY* addLandmarkPose(double x, double y, int strength = 1);
		
		g2o::VertexSE2Prior* addPriorLandmarkPose(const g2o::SE2& se2);
		g2o::VertexSE2Prior* addPriorLandmarkPose(const Eigen::Vector3d& lan);
		g2o::VertexSE2Prior* addPriorLandmarkPose(double x, double y, double theta);
		
		
		/** FUNCTION TO ADD THE EGDES **/
		g2o::EdgeSE2* addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix3d& information);
		g2o::EdgeSE2* addOdometry(const g2o::SE2& observ, int from_id, int toward_id, const Eigen::Matrix3d& information);
		g2o::EdgeSE2* addOdometry(double x, double y, double theta, int from_id, int toward_id, const Eigen::Matrix3d& information);
		g2o::EdgeSE2* addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		g2o::EdgeSE2* addOdometry(const g2o::SE2& observ, int from_id, int toward_id);
		g2o::EdgeSE2* addOdometry(double x, double y, double theta, int from_id, int toward_id);
		
		g2o::EdgeSE2PointXY* addLandmarkObservation(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		g2o::EdgeSE2PointXY* addLandmarkObservation(const g2o::Vector2D& pos, int from_id, int toward_id);
		
		g2o::EdgeSE2Prior_malcolm* addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
// 		void addEdgePrior(g2o::SE2 observ, int from, int toward);
// 		void addEdgePrior(double x, double y, double theta, int from, int toward);
		
		g2o::EdgeLinkXY_malcolm* addLinkBetweenMaps(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		g2o::EdgeLinkXY_malcolm* addLinkBetweenMaps(const g2o::Vector2D& pos, int from_id, int toward_id);
		
		void removeLinkBetweenMaps(g2o::EdgeLinkXY_malcolm* v1);
		
		//FUNCTION TO REMOVE A VERTEX
		void removeVertex(g2o::HyperGraph::Vertex* v1);
		
		int findRobotNode(g2o::HyperGraph::Vertex* v);
		int findLandmarkNode(g2o::HyperGraph::Vertex* v);
		int findPriorNode(g2o::HyperGraph::Vertex* v);
		
		
		//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
		
		/**
		 * @brief Directly use the prior graph to init the prior part of the ACG
		 * 
		 */
		void addPriorGraph(const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>& graph);
		
		
		
		void copyNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph){
			
			//ATTENTION : Might crash
			if(_ndt_graph->wasInit()){
				delete _ndt_graph;
			}
			_ndt_graph = new ndt_feature::NDTFeatureGraph(ndt_graph);
			
		}
		
		
		
		void updateNDTGraph(){
			updateNDTGraph(*_ndt_graph);
		}
		
		/**
		 * @brief : take the NDT graph and update the NDT corners by adding every new node since last time and all new observations.
		 */
		void updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph);
		
		
		void init(){
			_optimizable_graph.init();
		}
		
		void initialGuess(){
			_optimizable_graph.computeInitialGuess();
		}
		
		void optimize(){
			_optimizable_graph.optimize();
		}
		
		
		/*******************' TESTING FUNCTION ********************/
		void printGraph(){

			auto idmap = _optimizable_graph.vertices();
			auto idmapedges = _optimizable_graph.edges();
			
			std::cout << std::endl;
			for ( auto it = idmap.begin(); it != idmap.end(); ++it ){
				std::cout << "id " << it->first<< std::endl;								
			}
			std::cout << std::endl;
			for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
				std::cout << " " << *ite << " connected";
				for(auto ite2 = (*ite)->vertices().begin(); ite2 != (*ite)->vertices().end() ; ++ite2){
					std::cout << " " << *ite2;
				}
				std::cout << std::endl;
			}		
		}
		
	private:
		
		
		//TODO
		void updateLinksAfterNDTGraph(const std::vector<g2o::VertexPointXY*>& new_landmarks); 
		
		
	
	};
}

}
#endif