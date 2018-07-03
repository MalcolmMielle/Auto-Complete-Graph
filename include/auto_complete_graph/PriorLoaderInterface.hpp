#ifndef AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016
#define AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016

#include <random>

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
#include "vodigrex/linefollower/LineFollowerGraphCorners.hpp"
// #include "KeypointPriorDetector.hpp"
#include "bettergraph/conversion.hpp"

namespace AASS{
	namespace acg{


		class PriorAttr : public AASS::vodigrex::SimpleNode{
		public:
			cv::KeyPoint keypoint;
			cv::Point2d position;
			cv::Mat descriptor;
			PriorAttr() : AASS::vodigrex::SimpleNode(){}
			PriorAttr(const AASS::vodigrex::SimpleNode& inp) : AASS::vodigrex::SimpleNode(inp){}
		};

		

		/**
		 * @brief The class with all the hardcoded first guess things
		 */
		class PriorLoaderInterface{
		public : 
			typedef typename bettergraph::SimpleGraph<PriorAttr, AASS::vodigrex::SimpleEdge> PriorGraph;
			typedef typename bettergraph::SimpleGraph<PriorAttr, AASS::vodigrex::SimpleEdge>::GraphType PriorGraphType;
			typedef typename boost::graph_traits<PriorGraphType>::vertex_iterator PriorVertexIterator;
			typedef typename boost::graph_traits<PriorGraphType>::vertex_descriptor PriorVertex;
			typedef typename boost::graph_traits<PriorGraphType>::edge_descriptor PriorEdge;
			typedef typename boost::graph_traits<PriorGraphType>::out_edge_iterator PriorEdgeIterator;
			
		protected:

			cv::Mat _img_gray;
// 			std::vector<cv::Point2f> _corner_prior;
			std::vector<cv::Point2f> _same_point_prior;
			std::vector<cv::Point2f> _same_point_slam;
			std::vector<cv::Point2f> _corner_prior_matched;
			PriorGraph _prior_graph;
			cv::Mat _scale_transform_prior2ndt;
			std::string _file;
			double _deviation;
			double _angle;
			double _scale;
			double _max_deviation_for_corner;
			cv::Point2f _center;
			
// 			KeypointPriorDetector _keypoints;
			
// 			bool _extractKeypoints;
			
		public: 
			PriorLoaderInterface(const std::string& file) : _file(file), _max_deviation_for_corner((45 * M_PI) / 180)
//             , _extractKeypoints(false)
            {
			}

			PriorLoaderInterface(const std::string& file, double deviation, double anglet, double scalet, cv::Point2f center) : _file(file), _max_deviation_for_corner((45 * M_PI) / 180)
//             , _extractKeypoints(false)
			{
				std::vector<cv::Point2f> pt_slam;
				std::vector<cv::Point2f> pt_prior;

//				//Orebro
				pt_slam.push_back(cv::Point2f(19.36, 6.25));
				pt_prior.push_back(cv::Point2f(786, 373));
//
				pt_slam.push_back(cv::Point2f(19.14, 2.25));
				pt_prior.push_back(cv::Point2f(788, 311));


				//Dortmund
//				pt_slam.push_back(cv::Point2f(5, 0.55));
//				pt_prior.push_back(cv::Point2f(2429, 736));

//				pt_slam.push_back(cv::Point2f(4.86, -2.98));
//				pt_prior.push_back(cv::Point2f(2817, 748));

// 				pt_slam.push_back(cv::Point2f(7.64, 3.63));
// 				pt_prior.push_back(cv::Point2f(614, 306));
//
// 				pt_slam.push_back(cv::Point2f(9.5, 16));
// 				pt_prior.push_back(cv::Point2f(637, 529));

				initialize(pt_slam, pt_prior, deviation, anglet, scalet, center);

			}

			void setMaxDeviationForCornerInRad(double setter){_max_deviation_for_corner = setter;}
			
// 			void extractSIFTFeature(bool ext){_extractKeypoints = ext;}
// 			bool useFeatures(){return _extractKeypoints;}
			
			const PriorGraph getGraph() const {return _prior_graph;}
			PriorGraph getGraph(){return _prior_graph;}
			
			double getDeviation(){return _deviation;}
			double getAngle(){return _angle;}
			double getScale(){return _scale;}
			cv::Point2f getCenter(){return _center;}
			
			/**
			 * @brief Extract the corners and do the transformation onto the SLAM
			 */
			void prepare(){
				extractCornerPrior();
				transformOntoSLAM();
			}
			
			/**
			* @brief Extracting the corner from the prior
			*/
			void extractCornerPrior();
			
			/**
			 * @brief scale the corner and the graph depending on the _scale_transform_prior2ndt attribute
			 */
			void transformOntoSLAM();
			
			/**
			 * @brief initialze the structure
			 * @param[in] pt_slam : points from the robot slam map
			 * @param[in] pt_prior : equivalent points from the robot slam map
			 */
			void initialize(const std::vector<cv::Point2f>& pt_slam, const std::vector<cv::Point2f>& pt_prior, double deviation, double angle, double scale, cv::Point2f center){
				
				_deviation = deviation;
				_angle = angle;
				_scale = scale;
				_center = center;
				
				initialize(pt_slam, pt_prior);
				
			}
			
			/**
			 * @brief initialze the structure
			 * @param[in] pt_slam : points from the robot slam map
			 * @param[in] pt_prior : equivalent points from the robot slam map
			 */
			void initialize(const std::vector<cv::Point2f>& pt_slam, const std::vector<cv::Point2f>& pt_prior);
			
			
		protected:
			
		
			
			
			void extractKeypoints(const PriorGraph& graph);
			
			
			void rotateGraph(const cv::Mat& rot_mat );
			
			void AffineTransformGraph(const cv::Mat& warp_transfo );
			
			
			void noTwiceSameEdge(bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> graph);
			
			/**
			 * remove eahc edges in between same vertex
			 */
			void toSimpleGraph(bettergraph::PseudoGraph <AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge >& prior);
			  
			
			void convertGraph(const bettergraph::PseudoGraph <AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge >& input, bettergraph::PseudoGraph <PriorAttr, AASS::vodigrex::SimpleEdge>& output);
			
			
			
		};
		
	}
}

#endif
