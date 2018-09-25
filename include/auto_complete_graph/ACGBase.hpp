
#ifndef AUTOCOMPLETEGRAPH_ACGBASE_15092016
#define AUTOCOMPLETEGRAPH_ACGBASE_15092016

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
#include "VertexAndEdge/EdgeSE2Prior.hpp"
#include "VertexAndEdge/VertexLandmarkNDT.hpp"
#include "VertexAndEdge/VertexSE2Prior.hpp"
#include "VertexAndEdge/VertexSE2RobotPose.hpp"

#include "ACGPriorSE2.hpp"
#include "auto_complete_graph/Localization/ACGPriorXY.hpp"


namespace AASS {

namespace acg{

	/**
		 * @brief : used in a function to update the NDTGraph
		 * */
	class NDTCornerGraphElement : public perception_oru::ndt_feature_finder::NDTCornerBundle{
	public:
		cv::Point2f position_in_robot_frame;
	protected:
		//TODO : change it to a set
// 				std::vector<int> nodes_linked;
		g2o::VertexSE2RobotPose* nodes_linked_ptr;
		Eigen::Vector2d observations;
//				std::vector<double> _angle_orientation;
//				std::vector<double> _angle_width;

	public:
		std::vector<boost::shared_ptr< perception_oru::NDTCell > > cells1;
		std::vector<boost::shared_ptr< perception_oru::NDTCell > > cells2;

//				NDTCornerGraphElement(float x, float y) : position_in_robot_frame(x, y){};
		NDTCornerGraphElement(const cv::Point2f& p) : position_in_robot_frame(p){};
		NDTCornerGraphElement(const cv::Point2f& p, const perception_oru::ndt_feature_finder::NDTCornerBundle& ndtcorner): perception_oru::ndt_feature_finder::NDTCornerBundle(ndtcorner), position_in_robot_frame(p) {};

		void addAllObserv(g2o::VertexSE2RobotPose* ptr, Eigen::Vector2d obs){
// 					nodes_linked.push_back(i);
			observations = obs;
			nodes_linked_ptr = ptr;
//					_angle_orientation = angle_orientation;
//					_angle_width = a_width;
		}


// 				size_t size(){
// 					assert(nodes_linked.size() == nodes_linked_ptr.size());
// 					assert(nodes_linked.size() == observations.size());
// 					return observations.size();
// 				}

		// 		std::vector<int>& getNodeLinked(){return nodes_linked;}
// 				const std::vector<int>& getNodeLinked() const {return nodes_linked;}
		g2o::VertexSE2RobotPose* getNodeLinkedPtr() {return nodes_linked_ptr;}
		const Eigen::Vector2d& getObservations() const {return observations;}
//				const std::vector<double>& getOrientations() const {return _angle_orientation;}
//				const std::vector<double>& getAngleWidths() const {return _angle_width;}

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
//	class EdgePriorAndInitialValue{
//	protected:
//		EdgePrior* _edge;
//		g2o::SE2 _original_value;
//
//	public:
//		EdgePriorAndInitialValue(EdgePrior* ed, const g2o::SE2& orig_val) : _edge(ed), _original_value(orig_val){}
//
//		EdgePrior* getEdge(){return _edge;}
//		g2o::SE2 getOriginalValue(){return _original_value;}
//	};

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







	
	/**
	 * @brief The graph class containing all elemnts from each map and the graph used in the g2o optimisation.
	 * ATTENTION : _transNoise is not used anymore when update the NDT-graph for we used the registration. I should remove it but for now I'm leaving it only not to break everything.
	 */
	template< typename Prior, typename VertexPrior, typename EdgePrior >
	class AutoCompleteGraphBase{


	protected:


		double _z_elevation;

		int _previous_number_of_node_in_ndtgraph;



		///@brief USELESS NOW ! use the user inputted cov for the prior. Use the length of the edge if false
//		bool _use_user_prior_cov;

		///@brief user user inputted cov for robot pos. Uses registration otherwise
		bool _use_user_robot_pose_cov;

		bool _use_prior;

		bool _flag_optimize;

		bool _flag_use_robust_kernel = true;
		bool _flag_use_huber_kernel = true;
		bool _flag_use_dcs_kernel = true;

		bool _flag_use_corner_orientation;


		std::deque<double> _chi2s;

		///@brief Threshold of error mean under which we stop the optimization because we arrived to a stable result.
		double _error_threshold_stop_optimization;
		///@brief Number of optimizatino for which the error needs ot be stable to be considered finished.
		int _check_error_stable_over;

		//Needed system values
		Eigen::Vector2d _transNoise;
		double _rotNoise;
		Eigen::Vector2d _landmarkNoise;
		g2o::ParameterSE2Offset* _sensorOffset;
		g2o::SE2 _sensorOffsetTransf;


		///@brief the prior
//		AutoCompleteGraphPriorSE2* _prior;
		Prior* _prior;

//		RobotPoses* _robot_poses;

		///@brief vector storing all node from the landarks
		std::vector<g2o::VertexLandmarkNDT*> _nodes_landmark;
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<g2o::VertexSE2RobotPose*> _nodes_ndt;

		///@brief vector storing all edges between a landmark and the robot
		std::vector<g2o::EdgeLandmark_malcolm*> _edge_landmark;

		///@brief vector storing the odometry
		std::vector<g2o::EdgeOdometry_malcolm*> _edge_odometry;

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

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		AutoCompleteGraphBase(const g2o::SE2& sensoffset,
						const Eigen::Vector2d& tn,
						double rn,
						const Eigen::Vector2d& ln,
						const Eigen::Vector2d& pn,
						double rp,
						ndt_feature::NDTFeatureGraph* ndt_graph
  					) : _z_elevation(0), _use_user_robot_pose_cov(false), _use_prior(true), _sensorOffsetTransf(sensoffset), _transNoise(tn), _rotNoise(rn), _landmarkNoise(ln), _previous_number_of_node_in_ndtgraph(1), _optimizable_graph(sensoffset), _ndt_graph(ndt_graph), _first_Kernel_size(1), _age_step(0.1), _age_start_value(0.1), _flag_optimize(false), _flag_use_robust_kernel(true), _max_age(-1), _min_age(0), new_id_(0), _error_threshold_stop_optimization(1), _check_error_stable_over(10), _flag_use_corner_orientation(false), _prior(new Prior(pn, rp, sensoffset)) {
						// add the parameter representing the sensor offset ATTENTION was ist das ?
						_sensorOffset = new g2o::ParameterSE2Offset;
						_sensorOffset->setOffset(_sensorOffsetTransf);
						_sensorOffset->setId(0);
					}

		AutoCompleteGraphBase(const g2o::SE2& sensoffset,
						  const Eigen::Vector2d& tn,
						  double rn,
						  const Eigen::Vector2d& ln,
						  const Eigen::Vector2d& pn,
						  double rp
					) : _z_elevation(0), _use_user_robot_pose_cov(false), _use_prior(true), _sensorOffsetTransf(sensoffset), _transNoise(tn), _rotNoise(rn), _landmarkNoise(ln), _previous_number_of_node_in_ndtgraph(1), _optimizable_graph(sensoffset), _first_Kernel_size(1), _age_step(0.1), _age_start_value(0.1), _flag_optimize(false), _flag_use_robust_kernel(true), _max_age(-1), _min_age(0), new_id_(0), _error_threshold_stop_optimization(1), _check_error_stable_over(10), _flag_use_corner_orientation(false), _prior(new Prior(pn, rp, sensoffset)){

						// add the parameter representing the sensor offset ATTENTION was ist das ?
						_sensorOffset = new g2o::ParameterSE2Offset;
						_sensorOffset->setOffset(_sensorOffsetTransf);
						_sensorOffset->setId(0);
						_ndt_graph = NULL;

					}


		AutoCompleteGraphBase(const g2o::SE2& sensoffset, const std::string& load_file) : _z_elevation(0), _use_user_robot_pose_cov(false), _use_prior(true), _sensorOffsetTransf(sensoffset), _previous_number_of_node_in_ndtgraph(1), _optimizable_graph(sensoffset), _first_Kernel_size(1), _age_step(0.1), _age_start_value(0.1), _flag_optimize(false), _flag_use_robust_kernel(true), _max_age(-1), _min_age(0), new_id_(0), _error_threshold_stop_optimization(1), _check_error_stable_over(10), _flag_use_corner_orientation(false), _prior(new Prior(sensoffset)){


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
			_prior->setPriorNoise(a, b);
//			std::cout << _priorNoise << std::endl;
// 			assert(a == 1);
// 			assert(b == 0.01);
			infile >> a;
			_prior->setPriorRot(DEG2RAD(a));
			infile >> a >> b;
//			_linkNoise << a, b;
//			std::cout << _linkNoise << std::endl;
// 			assert(a == 0.2);
// 			assert(b == 0.2);

			infile >> _age_start_value; std::cout << _age_start_value << std::endl;
			infile >> _age_step; std::cout << _age_step << std::endl;
			infile >> _max_age; infile >> _min_age;
//			infile >> _min_distance_for_link_in_meter; std::cout << _min_distance_for_link_in_meter << std::endl;
//			infile >> _max_distance_for_link_in_meter; std::cout << _max_distance_for_link_in_meter << std::endl;
			infile >> a >> b;

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
		AutoCompleteGraphBase(const AutoCompleteGraphBase& that) = delete;

		virtual ~AutoCompleteGraphBase(){

// 			std::cout << "Calling ACG dest with size" << _nodes_ndt.size() << std::endl ;
			delete _sensorOffset;
			//The _optimizable_graph already delete the vertices in the destructor

// 			cleanup pointers in NODE

			for(size_t i = 0 ; i < _nodes_ndt.size() ; ++i){
// 				std::cout << "Deleting the NDT maps" << std::endl;
//  				delete _nodes_ndt[i].getMap();
			}

			_prior->clear();
			delete _prior;
// 			std::cout << "OUT ACG dest " << std::endl ;

		}


		virtual void print() const {


			std::cout << "Number of corners " << getLandmarkNodes().size() << std::endl;
			std::cout << "Number of poses " << getRobotNodes().size() << std::endl;
			std::cout << "Number of observation " <<getLandmarkEdges().size() << std::endl;

			std::cout << "using those parameters:" << std::endl;

			std::cout << "z elevation: " << _z_elevation << std::endl;
			std::cout << "previous nb of node in graph: " << _previous_number_of_node_in_ndtgraph << std::endl;

			///@brief USELESS NOW ! use the user inputted cov for the prior. Use the length of the edge if false
//		bool _use_user_prior_cov;

			///@brief user user inputted cov for robot pos. Uses registration otherwise
			std::cout << "use user robot pose covariance: " << _use_user_robot_pose_cov << std::endl;
			std::cout << "use prior: " << _use_prior << std::endl;
			std::cout << "optimize: " << _flag_optimize << std::endl;
			std::cout << "use robust kernel: " << _flag_use_robust_kernel << std::endl;
			std::cout << "use corner orientation " << _flag_use_corner_orientation << std::endl;

		}

		/** Accessor**/

		void setLandmarkNoise(const Eigen::Vector2d& co){_landmarkNoise = co;}
		void setLandmarkNoise(double landmark_noise_x, double landmark_noise_y){_landmarkNoise << landmark_noise_x, landmark_noise_y;}

		Prior* getPrior(){return _prior;}
		const Prior* getPrior() const {return _prior;}

//		std::set<VertexPrior*>& getNodes(){return _prior->getNodes();}
//		const std::set<VertexPrior*>& getNodes() const {return _prior->getNodes();}
		///@brief vector storing all node from the prior
		std::vector<g2o::VertexLandmarkNDT*>& getLandmarkNodes(){return _nodes_landmark;}
		const std::vector<g2o::VertexLandmarkNDT*>& getLandmarkNodes() const {return _nodes_landmark;}
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<g2o::VertexSE2RobotPose*>& getRobotNodes(){return _nodes_ndt;}
		const std::vector<g2o::VertexSE2RobotPose*>& getRobotNodes() const {return _nodes_ndt;}

		///@brief vector storing all edges between a landmark and the robot
		std::vector<g2o::EdgeLandmark_malcolm*>& getLandmarkEdges(){return _edge_landmark;}
		const std::vector<g2o::EdgeLandmark_malcolm*>& getLandmarkEdges() const {return _edge_landmark;}
		///@brief vector storing all edge between the prior nodes
//		std::vector<EdgePrior*>& getEdges(){ return _prior->getEdges();}
//		const std::vector<EdgePrior*>& getEdges() const { return _prior->getEdges();}
		///@brief vector storing the odometry
		std::vector<g2o::EdgeOdometry_malcolm*>& getOdometryEdges(){return _edge_odometry;}
		const std::vector<g2o::EdgeOdometry_malcolm*>& getOdometryEdges() const {return _edge_odometry;}

		void setZElevation(double z_set){_z_elevation = z_set;}
		double getZElevation() const {return _z_elevation;}

		void useUserCovForPrior(bool u){_prior->useUserCovForPrior(u);}
		bool isUsingUserCovForPrior() const {return _prior->isUsingUserCovForPrior();}

		void useUserCovForRobotPose(bool u){_use_user_robot_pose_cov = u;}
		bool isUsingUserCovForRobotPose() const {return _use_user_robot_pose_cov;}

		void useCornerOrientation(bool b){_flag_use_corner_orientation = b;}
		bool isUsingCornerOrientation() const {return _flag_use_corner_orientation;}

		void usePrior(bool setter){_use_prior = setter;}
		bool isUsingPrior() const {return _use_prior;}

		double getStepAge() const {return _age_step;}
		void setStepAge(double ss){_age_step = ss;}

		double getStartAge() const {return _age_start_value;}
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
			return true;
		}

		void useRobustKernel(bool use){_flag_use_robust_kernel = use;}
		void useHuberKernel(bool use){_flag_use_huber_kernel = use;}
		void useDCSKernel(bool use){_flag_use_dcs_kernel = use;}

		///@brief the main dish : the graph
		AASS::acg::OptimizableAutoCompleteGraph& getGraph(){return _optimizable_graph;}
		const AASS::acg::OptimizableAutoCompleteGraph& getGraph() const {return _optimizable_graph;}

		/***FUNCTIONS TO ADD THE NODES***/
		virtual g2o::VertexSE2RobotPose* addRobotPose(const g2o::SE2& se2, const Eigen::Affine3d& affine, const std::shared_ptr<perception_oru::NDTMap>& map);
		virtual g2o::VertexSE2RobotPose* addRobotPose(const Eigen::Vector3d& rob, const Eigen::Affine3d& affine, const std::shared_ptr<perception_oru::NDTMap>& map);
		virtual g2o::VertexSE2RobotPose* addRobotPose(double x, double y, double theta, const Eigen::Affine3d& affine, const std::shared_ptr<perception_oru::NDTMap>& map);

		virtual g2o::VertexLandmarkNDT* addLandmarkPose(const g2o::Vector2& estimate, const cv::Point2f& position, int strength = 1);
		virtual g2o::VertexLandmarkNDT* addLandmarkPose(double x, double y, const cv::Point2f& position, int strength = 1);

		virtual VertexPrior* addPriorLandmarkPose(const g2o::SE2& se2, const PriorAttr& priorAttr);
		virtual VertexPrior* addPriorLandmarkPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr);
		virtual VertexPrior* addPriorLandmarkPose(double x, double y, double theta, const PriorAttr& priorAttr);


		/** FUNCTION TO ADD THE EGDES **/
		virtual g2o::EdgeOdometry_malcolm* addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix3d& information_tmp);
		virtual g2o::EdgeOdometry_malcolm* addOdometry(const g2o::SE2& observ, int from_id, int toward_id, const Eigen::Matrix3d& information);
		virtual g2o::EdgeOdometry_malcolm* addOdometry(double x, double y, double theta, int from_id, int toward_id, const Eigen::Matrix3d& information);
		virtual g2o::EdgeOdometry_malcolm* addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		virtual g2o::EdgeOdometry_malcolm* addOdometry(const g2o::SE2& observ, int from_id, int toward_id);
		virtual g2o::EdgeOdometry_malcolm* addOdometry(double x, double y, double theta, int from_id, int toward_id);

		virtual g2o::EdgeLandmark_malcolm* addLandmarkObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix2d& covariance_landmark);
		virtual g2o::EdgeLandmark_malcolm* addLandmarkObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		virtual g2o::EdgeLandmark_malcolm* addLandmarkObservation(const g2o::Vector2& pos, int from_id, int toward_id);

		virtual EdgePrior* addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
// 		void addEdgePrior(g2o::SE2 observ, int from, int toward);
// 		void addEdgePrior(double x, double y, double theta, int from, int toward);

		//FUNCTION TO REMOVE A VERTEX
		void removeVertex(g2o::HyperGraph::Vertex* v1);
// 		void removeEdge(g2o::HyperGraph::Edge* v1);

		int findRobotNode(g2o::HyperGraph::Vertex* v);
		int findLandmarkNode(g2o::HyperGraph::Vertex* v);

		g2o::EdgeLandmark_malcolm* findObservation(const g2o::HyperGraph::Vertex* from, const g2o::HyperGraph::Vertex* toward) const {
			for(auto land_edge : _edge_landmark){
				if(from == land_edge->vertices()[0] && toward == land_edge->vertices()[1]){
					return land_edge;
				}
				else if(from == land_edge->vertices()[1] && toward == land_edge->vertices()[0]){
					return land_edge;
				}
			}
			return NULL;
		}

//		int findPriorNode(g2o::HyperGraph::Vertex* v);


		//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH

		/**
		 * @brief Directly use the prior graph to init the prior part of the ACG
		 *
		 */
		void addPriorGraph(const PriorLoaderInterface::PriorGraph& graph);

		///@remove the prior and all link edges
		void clearPrior();


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
		virtual void updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph, bool noise_flag = false, double deviation = 0.5);


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

//			std::cout << "Get final score" << std::endl;
			_optimizable_graph.computeActiveErrors();
//			std::cout << "Score : " << _optimizable_graph.chi2() << std::endl;
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


		virtual void setFirst(){
			g2o::OptimizableGraph::Vertex *fRobot;
			if(_nodes_ndt.size() > 0) {
				fRobot = _nodes_ndt[0];
			}
			else {
				fRobot = *(_prior->getNodes().begin());
			}
			_optimizable_graph.setFirst(fRobot);
		}

		/**
		 * @brief actual optimization loop. Make sure setFirst and prepare are called before. Use Huber first and then DCS.
		 * @param[in] max_iter maximum number of iteration for huber and DCS. Default at 100. Needs to be more than 2.
		 *
		 */
		virtual std::pair<int, int> optimize(int max_iter = 100);

		void setAgeingHuberKernel();

		void setAgeingDCSKernel();


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





		///@brief return the vector corresponding to the shift due to the optimization.
		Eigen::Vector3d getLastTransformation();

// 		getGridMap();

	public:

		void prepare();

		virtual int optimize_simple(int max_iter);

		/**
		 * @brief Check if the error in the graph was stable over time
		 * return true if it is and false if it was not
		 */
		bool ErrorStable(const std::deque<double>& _chi_kernel, int max_steps = 10);

		std::shared_ptr< perception_oru::NDTMap > addElementNDT(ndt_feature::NDTFeatureGraph& ndt_graph, const std::vector< ndt_feature::NDTFeatureLink >& links, int element, double deviation, g2o::VertexSE2RobotPose** robot_ptr, g2o::SE2& robot_pos);

		virtual void extractCornerNDTMap(const std::shared_ptr< perception_oru::NDTMap >& map, g2o::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos);


//		///@brief do createNewLinks and removeBadLinks
//		virtual void updateLinks();
//
//		///@brief create links between close by landmark and prior
//		virtual int createNewLinks() {throw std::runtime_error("createNewLinks base function not implemented. Do no use.");}
//
//		///@brief remove links between too far landmarks and prior
//		virtual void removeBadLinks() {throw std::runtime_error("createNewLinks base function not implemented. Do no use.");}

		void updatePriorEdgeCovariance();
		virtual void setKernelSizeDependingOnAge(g2o::OptimizableGraph::Edge* e, bool step);

		void testNoNanInPrior(const std::string& before = "no data") const ;
		virtual void testInfoNonNul(const std::string& before = "no data") const ;

		///@brief return true if ACG got more than or equal to 5 links. Make this better. Removed for testing now
		virtual bool checkAbleToOptimize(){
//			std::cout << "edge ndt size: " << _nodes_ndt.size() << std::endl;
//			if(_nodes_ndt.size() > 4){
				return true;
//			}
//			return false;
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

//		virtual int countLinkToMake() {throw std::runtime_error("createNewLinks base function not implemented. Do no use.");}
//
//		virtual void checkLinkNotTooBig() {throw std::runtime_error("createNewLinks base function not implemented. Do no use.");}

//		void checkNoRepeatingPriorEdge();


		void checkRobotPoseNotMoved(const std::string& when);

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
		virtual void getAllCornersNDTTranslatedToGlobalAndRobotFrame(const std::shared_ptr<perception_oru::NDTMap>& map, g2o::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos, std::vector<AASS::acg::NDTCornerGraphElement>& corners_end);




		//FOR TESTING

		virtual bool verifyInformationMatrices(bool verbose) const
		{
			bool allEdgeOk = true;
			Eigen::SelfAdjointEigenSolver<g2o::MatrixX> eigenSolver;
			for (g2o::OptimizableGraph::EdgeSet::const_iterator it = _optimizable_graph.edges().begin(); it != _optimizable_graph.edges().end(); ++it) {
				g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);
				g2o::MatrixX::MapType information(e->informationData(), e->dimension(), e->dimension());
				// test on symmetry
				bool isSymmetric = information.transpose() == information;
				bool okay = isSymmetric;
				if (isSymmetric) {
					// compute the eigenvalues
					eigenSolver.compute(information, Eigen::EigenvaluesOnly);
					bool isSPD = eigenSolver.eigenvalues()(0) >= 0.;
					okay = okay && isSPD;
				}
				allEdgeOk = allEdgeOk && okay;
				if (! okay) {
					if (verbose) {
						if (! isSymmetric)
							std::cerr << "Information Matrix for an edge is not symmetric:";
						else
							std::cerr << "Information Matrix for an edge is not SPD:";
						for (size_t i = 0; i < e->vertices().size(); ++i)
							std::cerr << " " << e->vertex(i)->id();
						if (isSymmetric)
							std::cerr << "\teigenvalues: " << eigenSolver.eigenvalues().transpose();
						std::cerr << " information :\n" << information << "\n" << std::endl;
					}
				}
			}
			return allEdgeOk;
		}



	};

	#include "ACGBase.tpp"



}

}
#endif
