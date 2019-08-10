#ifndef AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016
#define AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
#include "vodigrex/linefollower/LineFollowerGraphCorners.hpp"
#include "bettergraph/conversion.hpp"

namespace AASS{
	namespace acg{
		
		
		class PriorAttr : public AASS::vodigrex::SimpleNode{
		public:
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
			cv::Point2f _center;
						
			
		public: 
			PriorLoaderInterface(const std::string& file) : _file(file){		
			}
			
			
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
