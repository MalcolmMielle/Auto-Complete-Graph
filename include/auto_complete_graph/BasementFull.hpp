#ifndef AUTOCOMPLETEGRAPH_BASEMENTFULL_15112016
#define AUTOCOMPLETEGRAPH_BASEMENTFULL_15112016

#include <random>

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
// #include "das/CornerDetector.hpp"
#include "PriorLoaderInterface.hpp"

namespace AASS{
	namespace acg{

		/**
		 * @brief The class with all the hardcoded first guess things
		 */
		class BasementFull : public PriorLoaderInterface{
			
		protected:
			
		public: 

            BasementFull(double deviation, double anglet, double scalet, cv::Point2f center) :
//		        PriorLoaderInterface("/home/malcolm/ros_catkin_ws/lunar_ws/src/auto_complete_graph/tests/Dortmund/topo_floor.png"){
 			    PriorLoaderInterface("/home/malcolm/ros_catkin_ws/lunar_ws/src/auto_complete_graph/tests/emergbasement_flipped_nodoor.png"){

			
				
				std::vector<cv::Point2f> pt_slam;
				std::vector<cv::Point2f> pt_prior;
				
				pt_slam.push_back(cv::Point2f(19.36, 6.25));
				pt_prior.push_back(cv::Point2f(786, 373));
				
				pt_slam.push_back(cv::Point2f(19.14, 2.25));
				pt_prior.push_back(cv::Point2f(788, 311));
				
// 				pt_slam.push_back(cv::Point2f(7.64, 3.63));
// 				pt_prior.push_back(cv::Point2f(614, 306));
// 				
// 				pt_slam.push_back(cv::Point2f(9.5, 16));
// 				pt_prior.push_back(cv::Point2f(637, 529));
				
				initialize(pt_slam, pt_prior, deviation, anglet, scalet, center);
				
				
// 				this->_cornerDetect.setMinimumDeviationCorner( (85 * 3.14159) / 180 );
// 				
// 				auto randomNoise = [](double mean, double deviationt) -> double {
// 					std::default_random_engine engine{std::random_device()() };
// 					std::normal_distribution<double> dist(mean, deviationt);
// 					return dist(engine);
// 				};
// 				
// // 				cv::Point center = cv::Point(7, 08);
// 				double angle = anglet;
// 				double scale = scalet;
// 				
// 				cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, scale);
// 				std::cout << rot_mat << std::endl;
// // 				exit(0);
// 				
// 				auto rotatef = [](const cv::Mat& rot_mat, const cv::Point2d point) -> cv::Point2d {
// 					//Matrix multiplication
// 					cv::Mat point_m = (cv::Mat_<double>(3,1) << point.x, point.y, 1);
// 					cv::Mat mat_out = rot_mat * point_m;
// // 					std::cout << "Mat out " << mat_out << std::endl;
// 					cv::Point2d point_out;
// 					point_out.x = mat_out.at<double>(0);
// 					point_out.y = mat_out.at<double>(1);
// 					return point_out;
// 				};
// 				
// 				auto noise_x = randomNoise(0, deviation);
// 				auto noise_y = randomNoise(0, deviation);
// 				
// 				std::cout << "Noise X Y " << noise_x << " " << noise_y << std::endl;
// // 				exit(0);
// 				
// 				cv::Point2f out = cv::Point2f(19.36 + noise_x, 6.25 + noise_y);
// 				cv::Point2f slam_point = rotatef(rot_mat, out);
// 				
// 				_same_point_prior.push_back(cv::Point2f(786, 373));
// 				_same_point_slam.push_back(slam_point);
// 				
// 				noise_x = randomNoise(0, deviation);
// 				noise_y = randomNoise(0, deviation);
// 				out = cv::Point2f(19.14 + noise_x, 2.25 + noise_y);
// 				slam_point = rotatef(rot_mat, out);
// 				
// 				_same_point_prior.push_back(cv::Point2f(788, 311));                   
// 				_same_point_slam.push_back(slam_point);
// 				
// 				noise_x = randomNoise(0, deviation);
// 				noise_y = randomNoise(0, deviation);
// 				out = cv::Point2f(7.64 + noise_x, 3.63 + noise_y);
// 				slam_point = rotatef(rot_mat, out);
// 				
// 				//ATTENTION : next line or for used the points only
// 				
// 				_same_point_prior.push_back(cv::Point2f(614, 306));
// 				_same_point_slam.push_back(slam_point);
// 				
// 				noise_x = randomNoise(0, deviation);
// 				noise_y = randomNoise(0, deviation);
// 				out = cv::Point2f(9.5 + noise_x, 16 + noise_y);
// 				slam_point = rotatef(rot_mat, out);
// 				
// 				_same_point_prior.push_back(cv::Point2f(637, 529));              
// 				_same_point_slam.push_back(slam_point);
// 				
// // 				_same_point_prior.push_back(cv::Point2f(786, 373));
// // 				_same_point_slam.push_back(cv::Point2f(786, 373));
// // 				
// // 				_same_point_prior.push_back(cv::Point2f(788, 311));                   
// // 				_same_point_slam.push_back(cv::Point2f(786, 373));
// // 				
// // 				//ATTENTION : next line or for used the points only
// // 				
// // 				_same_point_prior.push_back(cv::Point2f(614, 306));
// // 				_same_point_slam.push_back(cv::Point2f(786, 373));
// // 				
// // 				_same_point_prior.push_back(cv::Point2f(637, 529));              
// // 				_same_point_slam.push_back(cv::Point2f(786, 373));
// 				
// 				_scale_transform_prior2ndt = cv::findHomography(_same_point_prior, _same_point_slam, CV_RANSAC, 3, cv::noArray());
		
			}
			
			
			


			
		};
		
	}
}

#endif
