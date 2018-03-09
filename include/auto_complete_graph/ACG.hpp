
#ifndef AUTOCOMPLETEGRAPH_ACG_15092016
#define AUTOCOMPLETEGRAPH_ACG_15092016

#include <ctime>
#include <fstream>
#include <random>

// #include "g2o/types/slam2d/vertex_se2.h"
// #include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
// #include "g2o/types/slam2d/edge_se2_prior.h"
//#include "g2o/types/slam2d/edge_se2_link.h"
// #include "g2o/types/slam2d/edge_landmark_se2.h"
//#include "g2o/types/slam2d/edge_link_xy.h"
// #include "g2o/types/slam2d/vertex_se2_prior.h"
//#include "g2o/types/slam2d/edge_landmark_malcolm.h"
//#include "g2o/types/slam2d/edge_odometry_malcolm.h"
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

// #include "das/AssociationInterface.hpp"
// #include "das/NDTCorner.hpp"
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

namespace acg{
	
	/**
	 * @brief The graph class containing all elemnts from each map and the graph used in the g2o optimisation.
	 * ATTENTION : _transNoise is not used anymore when update the NDT-graph for we used the registration. I should remove it but for now I'm leaving it only not to break everything.
	 */
	class AutoCompleteGraph{


	protected:
		int _previous_number_of_node_in_ndtgraph;

		///@brief Minimum distance from a prior corner to a NDT corner. THe distance is given in meter and is fixed at 2m in the constuctor
		double _min_distance_for_link_in_meter;

		double _max_distance_for_link_in_meter;

		///@brief USELESS NOW ! use the user inputted cov for the prior. Use the length of the edge if false
		bool _use_user_prior_cov;

		///@brief user user inputted cov for robot pos. Uses registration otherwise
		bool _use_user_robot_pose_cov;

		bool _flag_optimize;

		bool _flag_use_robust_kernel;

		bool _flag_use_corner_orientation;

		std::deque<double> _chi2s;

		///@brief Threshold of error mean under which we stop the optimization because we arrived to a stable result.
		double _error_threshold_stop_optimization;
		///@brief Number of optimizatino for which the error needs ot be stable to be considered finished.
		int _check_error_stable_over;

		/**
		 * @brief : used in a function to update the NDTGraph
		 * */
		class NDTCornerGraphElement{
			public:
				cv::Point2f position;
			protected:
				//TODO : change it to a set
// 				std::vector<int> nodes_linked;
				AASS::acg::VertexSE2RobotPose* nodes_linked_ptr;
				Eigen::Vector2d observations;
				std::vector<double> _angle_orientation;
				std::vector<double> _angle_width;

			public:
				std::vector<boost::shared_ptr< perception_oru::NDTCell > > cells1;
				std::vector<boost::shared_ptr< perception_oru::NDTCell > > cells2;

				NDTCornerGraphElement(float x, float y) : position(x, y){};
				NDTCornerGraphElement(const cv::Point2f& p) : position(p){};

				void addAllObserv(AASS::acg::VertexSE2RobotPose* ptr, Eigen::Vector2d obs, const std::vector<double>& angle_orientation, const std::vector<double>& a_width){
// 					nodes_linked.push_back(i);
					observations = obs;
					nodes_linked_ptr = ptr;
					_angle_orientation = angle_orientation;
					_angle_width = a_width;
				}


// 				size_t size(){
// 					assert(nodes_linked.size() == nodes_linked_ptr.size());
// 					assert(nodes_linked.size() == observations.size());
// 					return observations.size();
// 				}

		// 		std::vector<int>& getNodeLinked(){return nodes_linked;}
// 				const std::vector<int>& getNodeLinked() const {return nodes_linked;}
				AASS::acg::VertexSE2RobotPose* getNodeLinkedPtr() {return nodes_linked_ptr;}
				const Eigen::Vector2d& getObservations() const {return observations;}
				const std::vector<double>& getOrientations() const {return _angle_orientation;}
				const std::vector<double>& getAngleWidths() const {return _angle_width;}

		// 		void push_back(int i){nodes_linked.push_back(i);}

		// 		void addNode(int i){nodes_linked.push_back(i);}
		// 		void addObservation(const g2o::Vector2& obs){ observations.push_back(obs);}

// 				void fuse(const NDTCornerGraphElement& cor){
// 					for(size_t i = 0 ; i < cor.getNodeLinked().size() ; ++i){
// 						bool seen = false;
// 						for(size_t j = 0 ; j < nodes_linked.size() ; ++j){
// 							if(cor.getNodeLinked()[i] == nodes_linked[j]){
// 								seen = true;
// 							}
// 						}
// 						if(seen == false){
// 							nodes_linked.push_back(cor.getNodeLinked()[i]);
// 							observations.push_back(cor.getObservations()[i]);
// 							nodes_linked_ptr.push_back(cor.getNodeLinkedPtr()[i]);
// 						}
// 					}
// 				}
// 				void print() const {std::cout << point << " nodes : ";
//
// 					for(size_t i = 0 ; i < nodes_linked.size()  ; ++i){
// 						std::cout << nodes_linked[i] << " " ;
// 					}
//
// 				}

			};

		//ATTENTION Already useless
		class EdgePriorAndInitialValue{
		protected:
			AASS::acg::EdgeSE2Prior_malcolm* _edge;
			g2o::SE2 _original_value;

		public:
			EdgePriorAndInitialValue(AASS::acg::EdgeSE2Prior_malcolm* ed, const g2o::SE2& orig_val) : _edge(ed), _original_value(orig_val){}

			AASS::acg::EdgeSE2Prior_malcolm* getEdge(){return _edge;}
			g2o::SE2 getOriginalValue(){return _original_value;}
		};

		class EdgeInterface
		{
			protected:
			bool _flag_set_age;
			public:
			g2o::SE2 _malcolm_original_value;
			double _malcolm_age;

		//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			EdgeInterface() : _flag_set_age(false), _malcolm_age(1){};

			virtual g2o::SE2 getOriginalValue(){return _malcolm_original_value;}
			virtual void setOriginalValue(const g2o::SE2& orig_val){_malcolm_original_value = orig_val;}
			virtual bool manuallySetAge(){return _flag_set_age;}
			virtual double getAge(){return _malcolm_age;}
			virtual bool setAge(double a){
// 				std::cout << "Setting the age" << std::endl;
				_flag_set_age = true;
				assert(_flag_set_age == true);
				_malcolm_age = a;
				return _flag_set_age;
			}


		};


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
		std::vector<AASS::acg::VertexSE2Prior*> _nodes_prior;
		///@brief vector storing all node from the landarks
		std::vector<AASS::acg::VertexLandmarkNDT*> _nodes_landmark;
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<AASS::acg::VertexSE2RobotPose*> _nodes_ndt;
		///@brief vector storing all linking edges
		std::vector<EdgeLinkXY_malcolm*> _edge_link;

		//TODO: hack because fuck g2o node they don't work
		std::vector<EdgeInterface> _edge_interface_of_links;

		///@brief vector storing all edges between a landmark and the robot
		std::vector<EdgeLandmark_malcolm*> _edge_landmark;
		///@brief vector storing all edge between the prior nodes
		std::vector<AASS::acg::EdgeSE2Prior_malcolm*> _edge_prior;
		///@brief vector storing the odometry
		std::vector<EdgeOdometry_malcolm*> _edge_odometry;

		///@brief the main dish : the graph
// 		g2o::OptimizableGraph _optimizable_graph;
		AASS::acg::OptimizableAutoCompleteGraph _optimizable_graph;

		//ATTENTION : I should avoid that if I want to run both thread at the same time since no copy is made. I should instead copy it
		ndt_feature::NDTFeatureGraph* _ndt_graph;

// 		std::vector < NDTCornerGraphElement > _ndt_corners;
		double _first_Kernel_size;

		double _age_step;
		double _age_start_value;
		///@brief max value of the age of an edge. if -1 age can be infinetly old. Need to be more or equal to 0
		double _max_age;
		///@brief min value of the age of an edge. Need to be more or equal to 0
		double _min_age;

		//Iterator on ID
		int new_id_;

	public:

		AutoCompleteGraph(const g2o::SE2& sensoffset,
						const Eigen::Vector2d& tn,
						double rn,
						const Eigen::Vector2d& ln,
						const Eigen::Vector2d& pn,
						double rp,
						const Eigen::Vector2d& linkn,
						ndt_feature::NDTFeatureGraph* ndt_graph
  					) : _use_user_prior_cov(false), _use_user_robot_pose_cov(false), _sensorOffsetTransf(sensoffset), _transNoise(tn), _rotNoise(rn), _landmarkNoise(ln), _priorNoise(pn), _prior_rot(rp), _linkNoise(linkn), _previous_number_of_node_in_ndtgraph(1), _min_distance_for_link_in_meter(1.5), _max_distance_for_link_in_meter(3), _optimizable_graph(sensoffset), _ndt_graph(ndt_graph), _first_Kernel_size(1), _age_step(0.1), _age_start_value(0.1), _flag_optimize(false), _flag_use_robust_kernel(true), _max_age(-1), _min_age(0), new_id_(0), _error_threshold_stop_optimization(1), _check_error_stable_over(10), _flag_use_corner_orientation(false) {
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
					) : _use_user_prior_cov(false), _use_user_robot_pose_cov(false), _sensorOffsetTransf(sensoffset), _transNoise(tn), _rotNoise(rn), _landmarkNoise(ln), _priorNoise(pn), _prior_rot(rp), _linkNoise(linkn), _previous_number_of_node_in_ndtgraph(1), _min_distance_for_link_in_meter(1.5), _max_distance_for_link_in_meter(3), _optimizable_graph(sensoffset), _first_Kernel_size(1), _age_step(0.1), _age_start_value(0.1), _flag_optimize(false), _flag_use_robust_kernel(true), _max_age(-1), _min_age(0), new_id_(0), _error_threshold_stop_optimization(1), _check_error_stable_over(10), _flag_use_corner_orientation(false){

						// add the parameter representing the sensor offset ATTENTION was ist das ?
						_sensorOffset = new g2o::ParameterSE2Offset;
						_sensorOffset->setOffset(_sensorOffsetTransf);
						_sensorOffset->setId(0);
						_ndt_graph = NULL;

					}


		AutoCompleteGraph(const g2o::SE2& sensoffset, const std::string& load_file) : _use_user_prior_cov(false), _use_user_robot_pose_cov(false), _sensorOffsetTransf(sensoffset), _previous_number_of_node_in_ndtgraph(1), _min_distance_for_link_in_meter(1.5), _max_distance_for_link_in_meter(3), _optimizable_graph(sensoffset), _first_Kernel_size(1), _age_step(0.1), _age_start_value(0.1), _flag_optimize(false), _flag_use_robust_kernel(true), _max_age(-1), _min_age(0), new_id_(0), _error_threshold_stop_optimization(1), _check_error_stable_over(10), _flag_use_corner_orientation(false){


			std::ifstream infile(load_file);

			double a, b, c;
// 			infile >> a >> b >> c;
// 			const g2o::SE2 sensoffset(a, b, c);
// 			_sensorOffsetTransf = sensoffset;
			infile >> a >> b;
			_transNoise << a, b;
// 			assert(a == 0.0005);
// 			assert(b == 0.0001);
			std::cout << _transNoise << std::endl;
			infile >> a;
			_rotNoise = DEG2RAD(a);
			std::cout << "Rot" << _rotNoise << std::endl;
// 			assert(_rotNoise == 2);
			infile >> a >> b;
			_landmarkNoise << a, b;
			std::cout << _landmarkNoise << std::endl;
// 			assert(a == 0.05);
// 			assert(b == 0.05);
			infile >> a >> b;
			_priorNoise << a, b;
			std::cout << _priorNoise << std::endl;
// 			assert(a == 1);
// 			assert(b == 0.01);
			infile >> a;
			_prior_rot = DEG2RAD(a);
			infile >> a >> b;
			_linkNoise << a, b;
			std::cout << _linkNoise << std::endl;
// 			assert(a == 0.2);
// 			assert(b == 0.2);

			infile >> _age_start_value; std::cout << _age_start_value << std::endl;
			infile >> _age_step; std::cout << _age_step << std::endl;
			infile >> _max_age; infile >> _min_age;
			infile >> _min_distance_for_link_in_meter; std::cout << _min_distance_for_link_in_meter << std::endl;
			infile >> _max_distance_for_link_in_meter; std::cout << _max_distance_for_link_in_meter << std::endl;

			infile >> _flag_use_robust_kernel;
			infile >> _use_user_robot_pose_cov;

			assert(_age_start_value >= 0);
			assert(_max_age >= 0);

// 			exit(0);

			_sensorOffset = new g2o::ParameterSE2Offset;
			_sensorOffset->setOffset(_sensorOffsetTransf);
			_sensorOffset->setId(0);
			_ndt_graph = NULL;

			if(infile.eof() == true){
				throw std::runtime_error("NOT ENOUGH PARAMETERS IN FILE");
			}


		}

		//Forbid copy
		AutoCompleteGraph(const AutoCompleteGraph& that) = delete;

		virtual ~AutoCompleteGraph(){

// 			std::cout << "Calling ACG dest with size" << _nodes_ndt.size() << std::endl ;
			delete _sensorOffset;
			//The _optimizable_graph already delete the vertices in the destructor

// 			cleanup pointers in NODE

			for(size_t i = 0 ; i < _nodes_ndt.size() ; ++i){
// 				std::cout << "Deleting the NDT maps" << std::endl;
//  				delete _nodes_ndt[i].getMap();
			}
// 			std::cout << "OUT ACG dest " << std::endl ;

		}

		/** Accessor**/
		std::vector<AASS::acg::VertexSE2Prior*>& getPriorNodes(){return _nodes_prior;}
		const std::vector<AASS::acg::VertexSE2Prior*>& getPriorNodes() const {return _nodes_prior;}
		///@brief vector storing all node from the prior
		std::vector<AASS::acg::VertexLandmarkNDT*>& getLandmarkNodes(){return _nodes_landmark;}
		const std::vector<AASS::acg::VertexLandmarkNDT*>& getLandmarkNodes() const {return _nodes_landmark;}
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<AASS::acg::VertexSE2RobotPose*>& getRobotNodes(){return _nodes_ndt;}
		const std::vector<AASS::acg::VertexSE2RobotPose*>& getRobotNodes() const {return _nodes_ndt;}
		///@brief vector storing all linking edges
		std::vector<EdgeLinkXY_malcolm*>& getLinkEdges(){return _edge_link;}
		const std::vector<EdgeLinkXY_malcolm*>& getLinkEdges() const {return _edge_link;}
		///@brief vector storing all edges between a landmark and the robot
		std::vector<EdgeLandmark_malcolm*>& getLandmarkEdges(){return _edge_landmark;}
		const std::vector<EdgeLandmark_malcolm*>& getLandmarkEdges() const {return _edge_landmark;}
		///@brief vector storing all edge between the prior nodes
		std::vector<AASS::acg::EdgeSE2Prior_malcolm*>& getPriorEdges(){ return _edge_prior;}
		const std::vector<AASS::acg::EdgeSE2Prior_malcolm*>& getPriorEdges() const { return _edge_prior;}
		///@brief vector storing the odometry
		std::vector<EdgeOdometry_malcolm*>& getOdometryEdges(){return _edge_odometry;}
		const std::vector<EdgeOdometry_malcolm*>& getOdometryEdges() const {return _edge_odometry;}

		void setMinDistanceForLinksInMeters(double inpu){_min_distance_for_link_in_meter = inpu;}
		double getMinDistanceForLinksInMeters(){return _min_distance_for_link_in_meter;}

		void setMaxDistanceForLinksInMeters(double inpu){_max_distance_for_link_in_meter = inpu;}
		double getMaxDistanceForLinksInMeters(){return _max_distance_for_link_in_meter;}

		void useUserCovForPrior(bool u){_use_user_prior_cov = u;}
		bool isUsingUserCovForPrior(){return _use_user_prior_cov;}

		void useUserCovForRobotPose(bool u){_use_user_robot_pose_cov = u;}
		bool isUsingUserCovForRobotPose(){return _use_user_robot_pose_cov;}

		void useCornerOrientation(bool b){_flag_use_corner_orientation = b;}
		bool isUsingCornerOrientation(){return _flag_use_corner_orientation;}

		double getStepAge(){return _age_step;}
		void setStepAge(double ss){_age_step = ss;}

		double getStartAge(){return _age_start_value;}
		void setAgeStartValue(double ss){ _age_start_value = ss;}

		void numberOfIterationsToCheckForStabilityOfError(int e){
			_check_error_stable_over = e;
		}
		void errorUnderWhichWeStopTheOptimization(int min_error){
			_error_threshold_stop_optimization = min_error;
		}

		bool save(const std::string& file_outt){
			_optimizable_graph.save(file_outt.c_str());
			std::cout << "saved to " << file_outt << "\n";
		}

		void useRobustKernel(bool use){_flag_use_robust_kernel = use;}

		///@brief the main dish : the graph
		AASS::acg::OptimizableAutoCompleteGraph& getGraph(){return _optimizable_graph;}
		const AASS::acg::OptimizableAutoCompleteGraph& getGraph() const {return _optimizable_graph;}

		/***FUNCTIONS TO ADD THE NODES***/
		AASS::acg::VertexSE2RobotPose* addRobotPose(const g2o::SE2& se2, const Eigen::Affine3d& affine, const std::shared_ptr<perception_oru::NDTMap>& map);
		AASS::acg::VertexSE2RobotPose* addRobotPose(const Eigen::Vector3d& rob, const Eigen::Affine3d& affine, const std::shared_ptr<perception_oru::NDTMap>& map);
		AASS::acg::VertexSE2RobotPose* addRobotPose(double x, double y, double theta, const Eigen::Affine3d& affine, const std::shared_ptr<perception_oru::NDTMap>& map);

		AASS::acg::VertexLandmarkNDT* addLandmarkPose(const g2o::Vector2& estimate, const cv::Point2f& position, int strength = 1);
		AASS::acg::VertexLandmarkNDT* addLandmarkPose(double x, double y, const cv::Point2f& position, int strength = 1);

		AASS::acg::VertexSE2Prior* addPriorLandmarkPose(const g2o::SE2& se2, const PriorAttr& priorAttr);
		AASS::acg::VertexSE2Prior* addPriorLandmarkPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr);
		AASS::acg::VertexSE2Prior* addPriorLandmarkPose(double x, double y, double theta, const PriorAttr& priorAttr);


		/** FUNCTION TO ADD THE EGDES **/
		EdgeOdometry_malcolm* addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix3d& information_tmp);
		EdgeOdometry_malcolm* addOdometry(const g2o::SE2& observ, int from_id, int toward_id, const Eigen::Matrix3d& information);
		EdgeOdometry_malcolm* addOdometry(double x, double y, double theta, int from_id, int toward_id, const Eigen::Matrix3d& information);
		EdgeOdometry_malcolm* addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		EdgeOdometry_malcolm* addOdometry(const g2o::SE2& observ, int from_id, int toward_id);
		EdgeOdometry_malcolm* addOdometry(double x, double y, double theta, int from_id, int toward_id);

		EdgeLandmark_malcolm* addLandmarkObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix2d& covariance_landmark);
		EdgeLandmark_malcolm* addLandmarkObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		EdgeLandmark_malcolm* addLandmarkObservation(const g2o::Vector2& pos, int from_id, int toward_id);

		AASS::acg::EdgeSE2Prior_malcolm* addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
// 		void addEdgePrior(g2o::SE2 observ, int from, int toward);
// 		void addEdgePrior(double x, double y, double theta, int from, int toward);

		EdgeLinkXY_malcolm* addLinkBetweenMaps(const g2o::Vector2& pos, AASS::acg::VertexSE2Prior* v2, AASS::acg::VertexLandmarkNDT* v1);

//		EdgeLinkXY_malcolm* addLinkBetweenMaps(const g2o::Vector2& pos, int from_id, int toward_id);

		std::vector <EdgeLinkXY_malcolm* >::iterator removeLinkBetweenMaps(EdgeLinkXY_malcolm* v1);

		//FUNCTION TO REMOVE A VERTEX
		void removeVertex(g2o::HyperGraph::Vertex* v1);
// 		void removeEdge(g2o::HyperGraph::Edge* v1);

		int findRobotNode(g2o::HyperGraph::Vertex* v);
		int findLandmarkNode(g2o::HyperGraph::Vertex* v);
		int findPriorNode(g2o::HyperGraph::Vertex* v);


		//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH

		/**
		 * @brief Directly use the prior graph to init the prior part of the ACG
		 *
		 */
		void addPriorGraph(const PriorLoaderInterface::PriorGraph& graph);

		///@remove the prior and all link edges
		void clearPrior(){
			std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
			int i = 0;
			for(auto it = _nodes_prior.begin() ; it != _nodes_prior.end() ; ++it){

				for(auto it1 = it + 1 ; it1 != _nodes_prior.end() ;++it1){
					assert(*it != *it1);
					++i;
				}
			}


			for(auto it = _nodes_prior.begin() ; it != _nodes_prior.end() ;){
// 				auto it_tmp = it;
// 				assert(*it_tmp == *it);
// 				++it;
// 				assert(*it_tmp != *it);
// 				std::cout <<"removing the vertex " << *it << std::endl;
// 				if (it != _nodes_prior.end()) {
// 					std::cout <<"Done " << _nodes_prior.size() <<std::endl;
					//DIESNT WORK :(
// 					this->removeVertex(*it);
					_optimizable_graph.removeVertex(*it, false);
					it = _nodes_prior.erase(it);
// 					std::cout <<"removed the vertex " << std::endl;
// 				}
// 				it = it_tmp;
// 				++it;

			}
// 			std::cout <<"Done final " << _nodes_prior.size() << " i " << i <<std::endl;
			assert(_nodes_prior.size() == 0);

// 			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
// 				_optimizable_graph.removeVertex(it, true);
// 			}
// 			std::cout <<"clearing the edges " << std::endl;
			 _edge_prior.clear();
			 _edge_link.clear();
			 _edge_interface_of_links.clear();

			//Making sure all edge prior were removed.
			auto idmapedges = _optimizable_graph.edges();
			for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "pointer " << dynamic_cast<AASS::acg::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
				assert( dynamic_cast<AASS::acg::EdgeSE2Prior_malcolm*>(*ite) == NULL );
				assert( dynamic_cast<EdgeLinkXY_malcolm*>(*ite) == NULL );
			}
			std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
			std::cout << "DONE removing " << std::endl;
		}


		void clearLinks(){
			std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
			int i = 0;

			for(auto it = _edge_link.begin() ; it != _edge_link.end() ;){

				_optimizable_graph.removeEdge(*it);
				it = _edge_link.erase(it);

			}
// 			std::cout <<"Done final " << _nodes_prior.size() << " i " << i <<std::endl;
			assert(_edge_link.size() == 0);

// 			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
// 				_optimizable_graph.removeVertex(it, true);
// 			}
// 			std::cout <<"clearing the edges " << std::endl;

			//Making sure all edge prior were removed.
			auto idmapedges = _optimizable_graph.edges();
			for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "pointer " << dynamic_cast<AASS::acg::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
				assert( dynamic_cast<EdgeLinkXY_malcolm*>(*ite) == NULL );
			}
			std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
			std::cout << "DONE removing " << std::endl;
		}


		void copyNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph){
			assert(true == false && " do not use");
			//ATTENTION : Might crash
			if(_ndt_graph->wasInit()){
				delete _ndt_graph;
			}
			_ndt_graph = new ndt_feature::NDTFeatureGraph(ndt_graph);

		}


// 		void clear(){
//
// 		}

		/**
		 * @brief update the NDTGraph from 0 and add noise to the edges depending and noise_percentage
		 * ATTENTION not needed! Just as to create a clear() function and add noise_percentage as an optionnal argument (default is 0)
		 */
// 		void updateFullNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph, double noise_percentage){};

		/**
		 * @brief Incrementally update the NDTGraph
		 * Only the new nodes are added to the graph. If the g2o graph has 4 nodes, only nodes 5 to last node of the NDT graph are added to it.
		 * Add NDT-corners and Robot poses.
		 */
		void updateNDTGraph(){
			updateNDTGraph(*_ndt_graph);
		}


		/**
		 * @brief Incrementally update the NDTGraph
		 * Only the new nodes are added to the graph. If the g2o graph has 4 nodes, only nodes 5 to last node of the NDT graph are added to it.
		 * Add NDT-corners and Robot poses.
		 */
		void updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph, bool noise_flag = false, double deviation = 0.5);


		void initializeOptimization(){
			_optimizable_graph.initializeOptimization();
		}

		void computeInitialGuess(){
			_optimizable_graph.computeInitialGuess();
		}

		/**
		 * @brief save the error in the graph into _chi2s
		 */
		void saveErrorStep(){

			std::cout << "Get final score" << std::endl;
			_optimizable_graph.computeActiveErrors();
			std::cout << "Score : " << _optimizable_graph.chi2() << std::endl;
			_chi2s.push_back(_optimizable_graph.chi2());

		}

		/** Export the chi error into a file. " : " is written in between every error"
		 *
		 */
		void exportChi2s(){
			std::string file_out = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/chi2s_";
			std::ostringstream convert;   // stream used for the conversion
			convert << this->getRobotNodes().size();
			file_out = file_out + convert.str();
			file_out = file_out + ".txt";
			std::ofstream infile(file_out);
// 			int co = 0;
			for(auto it  = _chi2s.begin() ; it != _chi2s.end() ; ++it){
				infile << *it;
				infile << " : " ;
// 				++co;
// 				if(co == 10){
// 					infile << "\n" ;
// 				}
			}
			infile.close();
		}


		void setFirst(){
			g2o::OptimizableGraph::Vertex* fRobot = _nodes_ndt[0];
			_optimizable_graph.setFirst(fRobot);
		}

		/**
		 * @brief actual optimization loop. Make sure setFirst and prepare are called before. Use Huber first and then DCS.
		 * @param[in] max_iter maximum number of iteration for huber and DCS. Default at 100. Needs to be more than 2.
		 *
		 */
		virtual void optimize(int max_iter = 100){

			_chi2s.clear();

			std::cout << "BEFORE THE OPTIMIZATION BUT AFTER ADDING A NODE" << std::endl;
			overCheckLinks();

			/********** HUBER kernel ***********/

// 			_optimizable_graph.setHuberKernel();

			_flag_optimize = checkAbleToOptimize();

			if(_flag_optimize == true){
				std::cout << "OPTIMIZE" << std::endl;
// 				checkRobotPoseNotMoved("before opti");

				if(_flag_use_robust_kernel){
					setAgeingHuberKernel();
				}
// 				checkRobotPoseNotMoved("set age in huber kernel");
				testNoNanInPrior("set age in huber kernel");
				testInfoNonNul("set age in huber kernel");
				testNoNanInPrior("update prior edge cov");

				optimize_simple(max_iter);

// 				//Avoid overshoot of the cov
// 				for(size_t i = 0 ; i < iter ; ++i){
// 					_optimizable_graph.optimize(1);
// // 					checkRobotPoseNotMoved("optimized with huber");
// 					testNoNanInPrior("optimized with huber");
// 					testInfoNonNul("optimized with huber");
// 					//Update prior edge covariance
// // 					updatePriorEdgeCovariance();
// 					testNoNanInPrior("update prior edge cov after opti huber");
// 					saveErrorStep();
// 				}

				/********** DCS kernel ***********/
				if(_flag_use_robust_kernel){
					setAgeingDCSKernel();
				}
				testNoNanInPrior("set age in DCS kernel");

				optimize_simple(max_iter);

// 				for(size_t i = 0 ; i < iter*2 ; ++i){
// 					_optimizable_graph.optimize(1);
// // 					checkRobotPoseNotMoved("optimized with dcs");
// 					testNoNanInPrior("optimized with dcs");
// 					testInfoNonNul("optimized with dcs");
// 					//Update prior edge covariance
// // 					updatePriorEdgeCovariance();
// 					testNoNanInPrior("update prior edge cov after opti dcs");
// 					saveErrorStep();
// 				}


			}
			else{
				std::cout << "No Optimization :(" << std::endl;
			}

			std::cout << "AFTER THE OPTIMIZATION CREATE" << std::endl;
			int count = countLinkToMake();
			int count2 = createNewLinks();
// 			if(count != count2){
// 				std::cout << "Weird different detection" << std::endl;
// 				throw std::runtime_error("ARF NOT GOOD COUNT");
// 			}
// 			overCheckLinks();

			removeBadLinks();
			std::cout << "AFTER THE OPTIMIZATION REMOVE" << std::endl;
			overCheckLinks();

			exportChi2s();
// 			checkRobotPoseNotMoved("after opti");
// 			cv::Mat d;
// 			createDescriptorNDT(d);

		}

		void setAgeingHuberKernel(){

// 			for (SparseOptimizer::VertexIDMap::const_iterator it = this->vertices().begin(); it != this->vertices().end(); ++it) {
// 				OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
// 				v->setMarginalized(false);
// 			}

			auto idmapedges = _optimizable_graph.edges();
			for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "Robust Kern" << std::endl;
				g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*ite);
				auto huber = new g2o::RobustKernelHuber();
				e->setRobustKernel(huber);
				setKernelSizeDependingOnAge(e, true);
			}
		}

		void setAgeingDCSKernel(){
// 			for (SparseOptimizer::VertexIDMap::const_iterator it = this->vertices().begin(); it != this->vertices().end(); ++it) {
// 				OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
// 				v->setMarginalized(false);
// 			}

			auto idmapedges = _optimizable_graph.edges();
			for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "Robust Kern" << std::endl;
				g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*ite);
				auto dcs = new g2o::RobustKernelDCS();
				e->setRobustKernel(dcs);
				setKernelSizeDependingOnAge(e, false);
			}
		}

		///@brief Set Marginalized to false and do initializeOptimization
		void prepare(){
			_optimizable_graph.prepare();
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



		//Todo move in private
		bool linkAlreadyExist(AASS::acg::VertexLandmarkNDT* v_pt, AASS::acg::VertexSE2Prior* v_prior, std::vector< EdgeLinkXY_malcolm* >::iterator& it);
		bool linkAlreadyExist(AASS::acg::VertexLandmarkNDT* v_pt, AASS::acg::VertexSE2Prior* v_prior);
		bool noDoubleLinks();

		Eigen::Vector3d getLastTransformation();

// 		getGridMap();

	public:

		void optimize_simple(int max_iter){
			int count = 0;
			std::deque<double> _chi_kernel;
			for ( ; count < max_iter && count < 2 ; ++count){
				std::cout << "Optimizing because we need to " << std::endl;
				_optimizable_graph.optimize(1);
				_optimizable_graph.computeActiveErrors();
				_chi_kernel.push_back(_optimizable_graph.chi2());
				saveErrorStep();

				testNoNanInPrior("optimized with huber");
				testInfoNonNul("optimized with huber");
				testNoNanInPrior("update prior edge cov after opti huber");
			}
			if(max_iter >= 2){
				while(ErrorStable(_chi_kernel) == false && count < max_iter){
					count++;
					std::cout << "Optimizing until error stops it " << std::endl;
					_optimizable_graph.optimize(1);
					_optimizable_graph.computeActiveErrors();
					_chi_kernel.push_back(_optimizable_graph.chi2());
					saveErrorStep();

					testNoNanInPrior("optimized with huber");
					testInfoNonNul("optimized with huber");
					testNoNanInPrior("update prior edge cov after opti huber");
				}
			}
			int a;
			std::cout << "Enter anything to pass " <<count << std::endl;
			std::cin >> a;

		}

		/**
		 * @brief Check if the error in the graph was stable over time
		 * return true if it is and false if it was not
		 */
		bool ErrorStable(const std::deque<double>& _chi_kernel, int max_steps = 10){
			if(_chi_kernel.size() < 2){
				throw std::runtime_error("Cannot determine error difference with less than 2 optimization cycles.");
			}
			int max_steps_count = 0;
			std::vector<double> error_diff;
			double error_mean = 0;
			auto it = _chi_kernel.rbegin();
			++it;
			for(; it != _chi_kernel.rend() ; ++it ){
				if(max_steps_count == max_steps){
					break;
				}
				error_diff.push_back(std::abs( *it - *(it - 1)) );
				error_mean = error_mean + std::abs( (*it - *(it - 1)) );
				++max_steps_count;
			}
			error_mean = error_mean / max_steps_count;

			it = _chi_kernel.rbegin();
			++it;
			max_steps_count = 0;
			for(; it != _chi_kernel.rend() ; ++it ){
				if(max_steps_count == max_steps){
					break;
				}
				std::cout << std::abs( *it - *(it - 1) ) << " ";
				++max_steps_count;
			}
			std::cout << std::endl;
			std::cout << "Error is " << error_mean << "threshold is " << _error_threshold_stop_optimization << std::endl;
			if(error_mean < _error_threshold_stop_optimization){
				return true;
			}
			return false;
		}

		std::shared_ptr< perception_oru::NDTMap > addElementNDT(ndt_feature::NDTFeatureGraph& ndt_graph, const std::vector< ndt_feature::NDTFeatureLink >& links, int element, double deviation, AASS::acg::VertexSE2RobotPose** robot_ptr, g2o::SE2& robot_pos);

		virtual void extractCornerNDTMap(const std::shared_ptr< perception_oru::NDTMap >& map, AASS::acg::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos);


		///@brief do createNewLinks and removeBadLinks
		virtual void updateLinks();

		///@brief create links between close by landmark and prior
		virtual int createNewLinks();

		///@brief remove links between too far landmarks and prior
		void removeBadLinks();

		void updatePriorEdgeCovariance();
		void setKernelSizeDependingOnAge(g2o::OptimizableGraph::Edge* e, bool step);

		void testNoNanInPrior(const std::string& before = "no data") const ;
		void testInfoNonNul(const std::string& before = "no data") const ;

		///@brief return true if ACG got more than or equal to 5 links. Make this better.
		virtual bool checkAbleToOptimize(){
			std::cout << "edge link size: " << _edge_link.size() << std::endl;
			if(_edge_link.size() >= 6){
				return true;
			}
			return false;
		}

		virtual void overCheckLinks(){
			checkLinkNotForgotten();
			checkLinkNotTooBig();
		}

		virtual void checkLinkNotForgotten(){

// 			std::cout << "check forgotten links" << std::endl;
// 			auto it = _nodes_landmark.begin();
// 			for(it ; it != _nodes_landmark.end() ; it++){
// 				Eigen::Vector2d pose_landmark = (*it)->estimate();
// 				auto it_prior = _nodes_prior.begin();
// 				for(it_prior ; it_prior != _nodes_prior.end() ; ++it_prior){
//
// 					Eigen::Vector3d pose_tmp = (*it_prior)->estimate().toVector();
// 					Eigen::Vector2d pose_prior; pose_prior << pose_tmp(0), pose_tmp(1);
// 					double norm_tmp = (pose_prior - pose_landmark).norm();
//
// 					//Update the link
// 					if(norm_tmp <= _min_distance_for_link_in_meter){
// 						if(linkAlreadyExist(*it, *it_prior) == false){
// 							std::cout << "NORM" << norm_tmp << "min dist " << _min_distance_for_link_in_meter << " and max " << _min_distance_for_link_in_meter << std::endl;
// 							throw std::runtime_error("A small link was forgotten");
// 						}
// 					}
//
// 				}
//
// 			}
		}

		int countLinkToMake(){

			int count = 0;
			std::cout << "check forgotten links" << std::endl;
			auto it = _nodes_landmark.begin();
			for(it ; it != _nodes_landmark.end() ; it++){
				Eigen::Vector2d pose_landmark = (*it)->estimate();
				auto it_prior = _nodes_prior.begin();
				for(it_prior ; it_prior != _nodes_prior.end() ; ++it_prior){

					Eigen::Vector3d pose_tmp = (*it_prior)->estimate().toVector();
					Eigen::Vector2d pose_prior; pose_prior << pose_tmp(0), pose_tmp(1);
					double norm_tmp = (pose_prior - pose_landmark).norm();

					//Update the link
					if(norm_tmp <= _min_distance_for_link_in_meter){
						if(linkAlreadyExist(*it, *it_prior) == false){
							std::cout << "NORM" << norm_tmp << "min dist " << _min_distance_for_link_in_meter << " and max " << _min_distance_for_link_in_meter << std::endl;
							count++;
						}
					}

				}

			}

			return count;
		}

		void checkLinkNotTooBig(){
			std::cout << "check no big links" << std::endl;
			//Check if no small links are ledft out

			//Check if the link are not too big
			for(auto it_old_links = _edge_link.begin(); it_old_links != _edge_link.end() ;it_old_links++){

				std::vector<Eigen::Vector3d> vertex_out;

				assert((*it_old_links)->vertices().size() == 2);

				AASS::acg::VertexSE2Prior* ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it_old_links)->vertices()[0]);
				if(ptr == NULL){
					std::cout << ptr << " and " << (*it_old_links)->vertices()[0] << std::endl;
					throw std::runtime_error("Links do not have the good vertex type. Prior");
				}
				auto vertex = ptr->estimate().toVector();
				vertex_out.push_back(vertex);

				AASS::acg::VertexLandmarkNDT* ptr2 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>((*it_old_links)->vertices()[1]);
				if(ptr2 == NULL){
					throw std::runtime_error("Links do not have the good vertex type. Landmark");
				}
				auto vertex2 = ptr2->estimate();
				Eigen::Vector3d pose_prior; pose_prior << vertex2(0), vertex2(1), 0;
				vertex_out.push_back(pose_prior);


				assert(vertex_out.size() == 2);
				double norm = (vertex_out[0] - vertex_out[1]).norm();
				//Attention magic number
				if(norm > _max_distance_for_link_in_meter ){
					if(linkAlreadyExist(ptr2, ptr) == false){
						std::cout << "NORM" << norm << "min dist " << _min_distance_for_link_in_meter << " and max " << _min_distance_for_link_in_meter << std::endl;
						throw std::runtime_error("Big link still present :O");
					}
				}
			}


		}

		void checkNoRepeatingPriorEdge(){
			for(auto it_vertex = _nodes_prior.begin() ; it_vertex != _nodes_prior.end() ; ++it_vertex){
				std::vector<std::pair<double, double> > out;
	// 			std::cout << "edges " << std::endl;
				auto edges = (*it_vertex)->edges();
	// 			std::cout << "edges done " << std::endl;
				std::vector<AASS::acg::EdgeSE2Prior_malcolm*> edges_prior;

				for ( auto ite = edges.begin(); ite != edges.end(); ++ite ){
	// 				std::cout << "pointer " << dynamic_cast<AASS::acg::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
					AASS::acg::EdgeSE2Prior_malcolm* ptr = dynamic_cast<AASS::acg::EdgeSE2Prior_malcolm*>(*ite);
					if(ptr != NULL){
						//Make sure not pushed twice
						for(auto ite2 = edges_prior.begin(); ite2 != edges_prior.end(); ++ite2 ){
							assert(ptr != *ite2);
						}
	// 						std::cout << " pushed edges " << std::endl;
						edges_prior.push_back(ptr);
	// 						std::cout << "pushed edges done " << std::endl;
					}
				}
				for(auto it = edges_prior.begin() ; it != edges_prior.end() ; ++it){
					for(auto ite2 = it +1 ; ite2 != edges_prior.end() ; ++ite2 ){
						assert((*it)->getOrientation2D(**it_vertex) != (*ite2)->getOrientation2D(**it_vertex));
					}
				}
			}
			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
				for(auto ite2 = it +1 ; ite2 != _edge_prior.end() ; ++ite2 ){
					assert(it != ite2);
				}
			}
		}


		void checkRobotPoseNotMoved(const std::string& when){
			std::cout << "testing after " << when << std::endl;
			for(auto it = _nodes_ndt.begin() ; it != _nodes_ndt.end() ; ++it){
				int init_x = (*it)->initial_transfo.toVector()(0) ;
				int init_y = (*it)->initial_transfo.toVector()(1) ;
				int init_z = (*it)->initial_transfo.toVector()(2) * 10;

				int update_x = (*it)->estimate().toVector()(0) ;
				int update_y = (*it)->estimate().toVector()(1) ;
				int update_z = (*it)->estimate().toVector()(2) * 10;

				if(init_x != update_x || init_y != update_y || init_z != update_z){
					std::cout << " init "  << init_x << " "<< init_y << " "<< init_z <<  " == " << update_x << " "<< update_y << " "<< update_z <<  std::endl;
					throw std::runtime_error("MOVE BASE");
				}
			}

		}

		/**
		 * @rbief return the max on x and y of the prior
		 *
		 */
		void getExtremaPrior(double& size_x, double& size_y) const;

// 		void testNoNanInPrior();


		void printCellsNum() const {
			std::cout << "Cell numbers " << std::endl;
			int  i = 0;
			for(auto it = _nodes_ndt.begin() ; it != _nodes_ndt.end() ; ++it){
				std::cout << "For node " << i << " : " << (*it)->getMap()->getAllCells().size() << std::endl;;
			}
			std::cin >> i;
		}


		/**
		 * Returns all the corner found in the NDT map in NDTCornerGraphElement. The corner positions are in the global frame while the obervation are in the robot pose frame, as in needed by g2o
		 */
		void getAllCornersNDTTranslatedToGlobalAndRobotFrame(const std::shared_ptr<perception_oru::NDTMap>& map, AASS::acg::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos, std::vector<AASS::acg::AutoCompleteGraph::NDTCornerGraphElement>& corners_end);

	};
}

}
#endif
