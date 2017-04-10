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
			double _deviation;
			double _angle;
			double _scale;
			cv::Point2f _center;
			
		public: 
			PriorLoaderInterface(const std::string& file) : _file(file){		
			}
			
			const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> getGraph() const {return _prior_graph;}
			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> getGraph(){return _prior_graph;}
			
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
			void extractCornerPrior(){
		// 		AASS::das::BasementPriorLine basement;
				
				_corner_prior.clear();
				_prior_graph.clear();
				
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

				std::cout << "Point for transfo" << std::endl;
				std::cout << _same_point_prior[0] << " " << _same_point_slam[0] << std::endl << _same_point_prior[1] << " " << _same_point_slam[1] << std::endl << _same_point_prior[2] << " " << _same_point_slam[0] << std::endl;
								
				//Translate to origin
				cv::Point2d translation = - srcTri[0];
				double angle = 0;
				cv::Mat warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), translation.x, std::sin(angle), std::cos(angle), translation.y, 0, 0, 1);
				AffineTransformGraph(warp_mat);
				
				//Rotate
				Eigen::Vector2d vece1s; vece1s << srcTri[1].x - srcTri[0].x, srcTri[1].y - srcTri[0].y;
				Eigen::Vector2d vece2s; vece2s << dstTri[1].x - dstTri[0].x, dstTri[1].y - dstTri[0].y;
				
				angle = atan2(vece2s(1),vece2s(0)) - atan2(vece1s(1), vece1s(0));
				if (angle < 0) angle += 2 * M_PI;
				
// 				std::cout << "Angle : "<< angle << std::endl;
// 				angle = angle * 2 * 3.14157 / 360;
				warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1);
// 				std::cout << "mat " << warp_mat << std::endl;
				AffineTransformGraph(warp_mat);
				
				//Scale
				cv::Point2f vec1 = srcTri[1] - srcTri[0];
				cv::Point2f vec2 = dstTri[1] - dstTri[0];
				cv::Mat mat_transfo =  (cv::Mat_<float>(2,2) << vec2.x / vec1.x, 0, 0, vec2.y / vec1.y);
				double l1 = std::sqrt( ( vec1.x * vec1.x ) + ( vec1.y * vec1.y ) );
				double l2 = std::sqrt( ( vec2.x * vec2.x ) + ( vec2.y * vec2.y ) );
				double ratio = l2 / l1;
				
				std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
// 				int i = 0 ;
				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					//ATTENTION !
					cv::Point2d point;
					point.x = _prior_graph[v].getX();
					point.y = _prior_graph[v].getY();
					
					cv::Point2d point_out = point * ratio;

					_prior_graph[v].setX(point_out.x);
					_prior_graph[v].setY(point_out.y);
				}
// 				//Translate back
				translation = srcTri[0];
				angle = 0;
				warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), translation.x, std::sin(angle), std::cos(angle), translation.y, 0, 0, 1);
				AffineTransformGraph(warp_mat);
// 				
// 				//Translate to point
				translation = dstTri[0] - srcTri[0];
				angle = 0;
				warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), translation.x, std::sin(angle), std::cos(angle), translation.y, 0, 0, 1);
				AffineTransformGraph(warp_mat);

				
			}
			
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
			void initialize(const std::vector<cv::Point2f>& pt_slam, const std::vector<cv::Point2f>& pt_prior){
				
				assert(pt_slam.size() >= 4);
				assert(pt_prior.size() == pt_slam.size());
				
				std::cout << pt_prior[0] << " " << pt_slam[0] << std::endl << pt_prior[1] << " " << pt_slam[1] << std::endl << pt_prior[2] << " " << pt_slam[0] << std::endl;
				
				_same_point_prior.clear();
				_same_point_slam.clear();
				
				this->_cornerDetect.setMinimumDeviationCorner( (85 * 3.14159) / 180 );
				
				auto randomNoise = [](double mean, double deviationt) -> double {
					std::default_random_engine engine{std::random_device()() };
					std::normal_distribution<double> dist(mean, deviationt);
					return dist(engine);
				};
				
// 				cv::Point center = cv::Point(7, 08);
				
				cv::Mat rot_mat = cv::getRotationMatrix2D(_center, _angle, _scale);
				std::cout << rot_mat << std::endl;
// 				exit(0);
				
				auto rotatef = [](const cv::Mat& rot_mat, const cv::Point2d point) -> cv::Point2d {
					//Matrix multiplication
					cv::Mat point_m = (cv::Mat_<double>(3,1) << point.x, point.y, 1);
					cv::Mat mat_out = rot_mat * point_m;
// 					std::cout << "Mat out " << mat_out << std::endl;
					cv::Point2d point_out;
					point_out.x = mat_out.at<double>(0);
					point_out.y = mat_out.at<double>(1);
					return point_out;
				};
				
				auto noise_x = randomNoise(0, _deviation);
				auto noise_y = randomNoise(0, _deviation);
				
				std::cout << "Noise X Y " << noise_x << " " << noise_y << std::endl;
// 				exit(0);
				
				cv::Point2f out = cv::Point2f(pt_slam[0].x + noise_x, pt_slam[0].y + noise_y);
				cv::Point2f slam_point = rotatef(rot_mat, out);
				
				_same_point_prior.push_back(pt_prior[0]);
				_same_point_slam.push_back(slam_point);
				
				noise_x = randomNoise(0, _deviation);
				noise_y = randomNoise(0, _deviation);
				out = cv::Point2f(pt_slam[1].x + noise_x, pt_slam[1].y + noise_y);
				slam_point = rotatef(rot_mat, out);
				
				_same_point_prior.push_back(pt_prior[1]);                   
				_same_point_slam.push_back(slam_point);
				
				noise_x = randomNoise(0, _deviation);
				noise_y = randomNoise(0, _deviation);
				out = cv::Point2f(pt_slam[2].x + noise_x, pt_slam[2].y + noise_y);
				slam_point = rotatef(rot_mat, out);
				
				//ATTENTION : next line or for used the points only
				
				_same_point_prior.push_back(pt_prior[2]);
				_same_point_slam.push_back(slam_point);
				
				noise_x = randomNoise(0, _deviation);
				noise_y = randomNoise(0, _deviation);
				out = cv::Point2f(pt_slam[3].x + noise_x, pt_slam[3].y + noise_y);
				slam_point = rotatef(rot_mat, out);
				
				_same_point_prior.push_back(pt_prior[3]);              
				_same_point_slam.push_back(slam_point);
				
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
				
				_scale_transform_prior2ndt = cv::findHomography(_same_point_prior, _same_point_slam, CV_RANSAC, 3, cv::noArray());
				
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
					std::cout << "reutnr point " << point_out.x << " " << point_out.y << std::endl;
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
					
					cv::Point2d point_out = transf(warp_transfo, point);
					
					//Matrix multiplication
					
// 					std::cout << "Point out " << point_out << std::endl;
					
// 					std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					_prior_graph[v].setX(point_out.x);
					_prior_graph[v].setY(point_out.y);
					_corner_prior_matched.push_back(point_out);
// 					std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
// 					std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					++i;
				}
				
				
			}
			
		};
		
	}
}

#endif