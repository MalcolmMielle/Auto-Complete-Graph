#ifndef AUTOCOMPLETEGRAPH_BASEMENT_26102016
#define AUTOCOMPLETEGRAPH_BASEMENT_26102016

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
// #include "das/CornerDetector.hpp"
#include "PriorLoaderInterface.hpp"

namespace AASS{
	namespace acg{

		/**
		 * @brief The class with all the hardcoded first guess things
		 */
		class Basement: public PriorLoaderInterface{
			
		protected:
// 			std::vector<cv::Point2f> _corner_prior;
// 			std::vector<cv::Point2f> _same_point_prior;
// 			std::vector<cv::Point2f> _same_point_slam;
// 			std::vector<cv::Point2f> _corner_prior_matched;
// 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> _prior_graph;
// 			cv::Mat _scale_transform_prior2ndt;
// 			std::string _file;
			
		public: 
			Basement() : PriorLoaderInterface("/home/malcolm/ros_catkin_ws/indigo_ws/src/auto_complete_graph/tests/map_simple1.png"){
				
				_same_point_prior.push_back(cv::Point2f(287,99));
				_same_point_slam.push_back(cv::Point2f(19.36, 6.25));
				
				_same_point_prior.push_back(cv::Point2f(287,42));                   
				_same_point_slam.push_back(cv::Point2f(19.14,2.25));
				
				//ATTENTION : next line or for used the points only
				
// 				_same_point_prior.push_back(cv::Point2f(117,37));
// 				_same_point_slam.push_back(cv::Point2f(7.64,3.63));
// 				
// 				_same_point_prior.push_back(cv::Point2f(138, 260));              
// 				_same_point_slam.push_back(cv::Point2f(9.5, 16));
// 				
// 				_scale_transform_prior2ndt = cv::findHomography(_same_point_prior, _same_point_slam, CV_RANSAC, 3, cv::noArray());
		
			}
			
// 			const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> getGraph() const {return _prior_graph;}
// 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> getGraph(){return _prior_graph;}
// 			
// 			/**
// 			* @brief Extracting the corner from the prior
// 			*/
// 			void extractCornerPrior(){
// 		// 		AASS::das::BasementPriorLine basement;
// // 				cv::Mat src = cv::imread( _file, CV_LOAD_IMAGE_COLOR ), src_gray;
// // 				cv::cvtColor(src, src_gray, CV_RGB2GRAY );
// // 				
// // 				cv::threshold(src_gray, src_gray, 100, 255, src_gray.type());
// 				
// 				AASS::das::CornerDetector cornerDetect;
// 				cornerDetect.getFeaturesGraph(_file);
// // 				cornerDetect.removeClosePoints(20);
// 				_corner_prior = cornerDetect.getGraphPoint();
// 				_prior_graph = cornerDetect.getGraph(); 
// 				
// 			}
// 			
// 			/**
// 			 * @brief scale the corner and the graph depending on the _scale_transform_prior2ndt attribute
// 			 */
// 			void transformOntoSLAM(){
// 				std::cout << "Transform onto slam from scale : " << std::endl << _scale_transform_prior2ndt << std::endl;
// 				//Scale transform here:		
// 				//Then create association
// 				cv::perspectiveTransform( _corner_prior, _corner_prior_matched, _scale_transform_prior2ndt);
// 				
// 				
// 				//Scale graph
// 				std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
// 				//vertices access all the vertix
// 				//Classify them in order
// 	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
// 				int i = 0 ;
// 				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
// 	// 					std::cout << "going throught grph " << i << std::endl; ++i;
// 					AASS::das::CornerDetector::CornerVertex v = *vp.first;
// 					cv::Point2f p;
// 					std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
// 					std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
// 					_prior_graph[v].setX(_corner_prior_matched[i].x);
// 					_prior_graph[v].setY(_corner_prior_matched[i].y);
// 					std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
// 					++i;
// 				}
// 				
// 				
// 				
// 			}
			
		};
		
	}
}

#endif