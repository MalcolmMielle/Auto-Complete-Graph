#ifndef AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016
#define AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
#include "das/CornerDetector.hpp"

namespace AASS{
	namespace acg{

		/**
		 * @brief The class with all the hardcoded first guess things
		 */
		class PriorLoaderInterface{
			
		protected:
			std::vector<cv::Point2f> _corner_prior;
			std::vector<cv::Point2f> _same_point_prior;
			std::vector<cv::Point2f> _same_point_slam;
			std::vector<cv::Point2f> _corner_prior_matched;
			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> _prior_graph;
			cv::Mat _scale_transform_prior2ndt;
			std::string _file;
			AASS::das::CornerDetector _cornerDetect;
			
		public: 
			PriorLoaderInterface(const std::string& file) : _file(file){
				
// 				_same_point_prior.push_back(cv::Point2f(786, 373));
// 				_same_point_slam.push_back(cv::Point2f(190.36, 60.25));
// 				
// 				_same_point_prior.push_back(cv::Point2f(788, 311));                   
// 				_same_point_slam.push_back(cv::Point2f(190.14,20.25));
// 				
// 				//ATTENTION : next line or for used the points only
// 				
// 				_same_point_prior.push_back(cv::Point2f(614, 306));
// 				_same_point_slam.push_back(cv::Point2f(70.64,30.63));
// 				
// 				_same_point_prior.push_back(cv::Point2f(637, 529));              
// 				_same_point_slam.push_back(cv::Point2f(90.5, 160));
				
// 				_same_point_prior.push_back(cv::Point2f(786, 373));
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
// 				
// 				_same_point_prior.push_back(cv::Point2f(788, 311));                   
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
// 				
// 				//ATTENTION : next line or for used the points only
// 				
// 				_same_point_prior.push_back(cv::Point2f(614, 306));
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
// 				
// 				_same_point_prior.push_back(cv::Point2f(637, 529));              
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
				
// 				_scale_transform_prior2ndt = cv::findHomography(_same_point_prior, _same_point_slam, CV_RANSAC, 3, cv::noArray());
		
			}
			
			const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> getGraph() const {return _prior_graph;}
			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> getGraph(){return _prior_graph;}
			
			/**
			* @brief Extracting the corner from the prior
			*/
			void extractCornerPrior(){
		// 		AASS::das::BasementPriorLine basement;
				
				_cornerDetect.clear();
				_cornerDetect.getFeaturesGraph(_file);
// 				cornerDetect.removeClosePoints(20);
				_corner_prior = _cornerDetect.getGraphPoint();
				_prior_graph = _cornerDetect.getGraph();
				
// 				//PRINT
// 				cv::Mat src = cv::imread( _file, CV_LOAD_IMAGE_COLOR ), src_gray;
// 				cv::cvtColor(src, src_gray, CV_RGB2GRAY );
// // 				
// 				cv::threshold(src_gray, src_gray, 100, 255, src_gray.type());
// 				cv::Mat maa_3 = src_gray.clone();
// 				maa_3.setTo(cv::Scalar(0));
// 				AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(_prior_graph, maa_3);
// 				std::cout << "Number of nodes " << _prior_graph.getNumVertices() << std::endl;
// 				cv::imshow("graph", maa_3);
// 				cv::imshow("map", src_gray);
// 				cv::waitKey(0);
				
			}
			
			/**
			 * @brief scale the corner and the graph depending on the _scale_transform_prior2ndt attribute
			 */
			void transformOntoSLAM(){
// 				std::cout << "Transform onto slam from scale : " << std::endl << _scale_transform_prior2ndt << std::endl;
				//Scale transform here:		
				//Then create association
// 				cv::perspectiveTransform( _corner_prior, _corner_prior_matched, _scale_transform_prior2ndt);
				
				assert(_same_point_prior.size() >= 3);
				
				cv::Point2f srcTri[3];
				cv::Point2f dstTri[3];
				srcTri[0] = _same_point_prior[0];
				srcTri[1] = _same_point_prior[1];
				srcTri[2] = _same_point_prior[2];
                          
				dstTri[0] = _same_point_slam[0];
				dstTri[1] = _same_point_slam[1];
				dstTri[2] = _same_point_slam[2];
				
				cv::Mat warp_mat = cv::getAffineTransform( srcTri, dstTri );
				/// Compute a rotation matrix with respect to the center of the image
// 				cv::Point center = cv::Point( warp_dst.cols/2, warp_dst.rows/2 );
// 				double angle = -50.0;
// 				double scale = 0.6;

				/// Get the rotation matrix with the specifications above
// 				rot_mat = cv::getRotationMatrix2D( center, angle, scale );
				
				/// Apply the Affine Transform just found to the src image
// 				cv::Mat warp_dst = cv::imread( _file, 1 );
				
				std::cout << "Warp Affine" << warp_mat <<std::endl;
				//TODO Implement myself
// 				cv::warpAffine( _same_point_prior, warp_dst, warp_mat, warp_dst.size() );
				AffineTransformGraph(warp_mat);
				
				
				
// 				cv::namedWindow( "fuck", CV_WINDOW_AUTOSIZE );
// 				cv::imshow( "fuck", warp_dst );
				/** Rotating the image after Warp */

				

				/// Rotate the warped image
// 				cv::warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );
				
				
				//Scale graph
				std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
				int i = 0 ;
				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					cv::Point2f p;
					std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
					_prior_graph[v].setX(_corner_prior_matched[i].x);
					_prior_graph[v].setY(_corner_prior_matched[i].y);
					std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					++i;
				}
				
				
				
			}
			
			
		protected:
			
			void rotateGraph(const cv::Mat& rot_mat ){
				
				
				auto transf = [](const cv::Mat& rot_mat, const cv::Point2d point) -> cv::Point2d {
					//Matrix multiplication
					cv::Mat point_m = (cv::Mat_<double>(3,1) << point.x, point.y);
					cv::Mat mat_out = rot_mat * point_m;
// 					std::cout << "Mat out " << mat_out << std::endl;
					cv::Point2d point_out;
					point_out.x = mat_out.at<double>(0);
					point_out.y = mat_out.at<double>(1);
					return point_out;
				};
				
				
				
				
				std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
				int i = 0 ;
				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					//ATTENTION !
					cv::Point2d point;
					point.x = _prior_graph[v].getX();
					point.y = _prior_graph[v].getY();
					
					auto point_out = transf(rot_mat, point);
					
					//Matrix multiplication
					
					std::cout << "Point out " << point_out << std::endl;
					
					std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					_prior_graph[v].setX(point_out.x);
					_prior_graph[v].setY(point_out.y);
					_corner_prior_matched.push_back(point_out);
					std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
					std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					++i;
				}
				
				
			}
			
			void AffineTransformGraph(const cv::Mat& warp_transfo ){
				
				
				auto transf = [](const cv::Mat& warp_transfo, const cv::Point2d point) -> cv::Point2d {
					//Matrix multiplication
					cv::Mat point_m = (cv::Mat_<double>(3,1) << point.x, point.y, 1);
					cv::Mat mat_out = warp_transfo * point_m;
// 					std::cout << "Mat out " << mat_out << std::endl;
					cv::Point2d point_out;
					point_out.x = mat_out.at<double>(0);
					point_out.y = mat_out.at<double>(1);
					return point_out;
				};
				
				
				
				
				std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
				int i = 0 ;
				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					//ATTENTION !
					cv::Point2d point;
					point.x = _prior_graph[v].getX();
					point.y = _prior_graph[v].getY();
					
					auto point_out = transf(warp_transfo, point);
					
					//Matrix multiplication
					
					std::cout << "Point out " << point_out << std::endl;
					
					std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					_prior_graph[v].setX(point_out.x);
					_prior_graph[v].setY(point_out.y);
					_corner_prior_matched.push_back(point_out);
					std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
					std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					++i;
				}
				
				
			}
			
		};
		
	}
}

#endif