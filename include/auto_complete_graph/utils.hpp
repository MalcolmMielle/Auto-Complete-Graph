#ifndef AUTOCOMPLETEGRAPH_UTILS_11042017
#define AUTOCOMPLETEGRAPH_UTILS_11042017

#include <iostream>
#include "opencv2/core/core.hpp"
#include "Eigen/Core"
#include "g2o/types/slam2d/se2.h"

namespace AASS{
	namespace acg{


		/**
		 * @brief return the abgle between two vector in rad. clockwise
		 */
		inline double getAngle(const Eigen::Vector3d& base, const Eigen::Vector3d& toward)
		{

			// 	std::cout << "Minor Axis " << std::endl << vec << std::endl;
			// 	std::cout << "Compared to  " << std::endl << vec2 << std::endl;
			//dot product with vertical axis gives us the angle
			double d = base.dot(toward);
			double l = base.norm() * toward.norm();

			// 	std::cout << " d : " << d << " norm " << l << std::endl;
			double ac = d/l;
			ac = std::acos(ac);
			return ac;

		}



		inline std::string type2str(int type) {
			std::string r;

			uchar depth = type & CV_MAT_DEPTH_MASK;
			uchar chans = 1 + (type >> CV_CN_SHIFT);

			switch ( depth ) {
				case CV_8U:  r = "8U"; break;
				case CV_8S:  r = "8S"; break;
				case CV_16U: r = "16U"; break;
				case CV_16S: r = "16S"; break;
				case CV_32S: r = "32S"; break;
				case CV_32F: r = "32F"; break;
				case CV_64F: r = "64F"; break;
				default:     r = "User"; break;
			}

			r += "C";
			r += (chans+'0');

			return r;
		}
		
		
		/**
		 * @param[in] vec_in : landmark pose in sub map position -> x, y, theta
		 * @param[in] robot_pos : pose of the sub map in the global reference frame
		 */
		inline void translateFromRobotFrameToGlobalFrame(const Eigen::Vector3d& vec_input, const g2o::SE2& robot_frame_pose, Eigen::Vector3d& pose_inglobal_frame){
			
			//Node it beong to :
			g2o::SE2 robot_frame_se2 = g2o::SE2(robot_frame_pose);
			//Pose of landmark
			g2o::SE2 landmark_inrobotframe_se2(vec_input);
			//Composition
			g2o::SE2 landmark_globalframe = robot_frame_se2 * landmark_inrobotframe_se2;
//			Eigen::Vector3d landmark_robotframe_vector = landmark_robotframe.toVector();
			
			
			pose_inglobal_frame = landmark_globalframe.toVector();
			//Pose of landmark in global reference
// 			point_inglobal_frame.x = landmark_robotframe_vector(0);
// 			point_inglobal_frame.y = landmark_robotframe_vector(1);
					
// 			Eigen::Vector2d real_obs; real_obs << point_inglobal_frame.x, point_inglobal_frame.y;
			
			//Projecting real_obs into robot coordinate frame
			
// 			auto trueObservation_tmp = robot_frame_pose.inverse() * landmark_robotframe;
			
// 			std::cout << trueObservation_tmp.toVector() << "\n\n" << landmark_se2.toVector() << std::endl;
// 			assert(trueObservation_tmp.toVector() == landmark_se2.toVector());
			
// 			pose_inglobal_frame = trueObservation_tmp.toVector();
// 			Eigen::Vector2d observation; observation << pose_inglobal_frame(0), pose_inglobal_frame(1);
			
// 			Eigen::Vector2d trueObservation2d = robot_frame_pose.inverse() * real_obs;
// 			Eigen::Vector2d observation2d_test;
// 			observation2d_test = trueObservation2d;	
// 			double angle_landmark = pose_inglobal_frame(2);
			
// 			return vec_inglobal_frame;
		}


		inline double distancePointLine(const Eigen::Vector2d& point, const Eigen::Vector2d& p1_line, const Eigen::Vector2d& p2_line){

//			std::cout << "Getting the distance " <<std::endl;
			double distance_point_to_line = std::abs( ( (p2_line[1] - p1_line[1] ) * point[0] ) - ( (p2_line[0] - p1_line[0]) * point[1] ) + ( p2_line[0]*p1_line[1]) - (p2_line[1]*p1_line[0]) );
			distance_point_to_line = distance_point_to_line / std::sqrt( ( (p2_line[1] - p1_line[1]) * (p2_line[1] - p1_line[1]) ) + ( (p2_line[0] - p1_line[0]) * (p2_line[0] - p1_line[0]) ) );
//			std::cout << "Distance : " << distance_point_to_line << "points distance " << std::sqrt( ( (p2_line[1] - p1_line[1]) * (p2_line[1] - p1_line[1]) ) + ( (p2_line[0] - p1_line[0]) * (p2_line[0] - p1_line[0]) ) ) << " numerator " << std::norm( ( (p2_line[1] - p1_line[1] ) * point[0] ) - ( (p2_line[0] - p1_line[0]) * point[1] ) + ( p2_line[0]*p1_line[1]) - (p2_line[1]*p1_line[0]) ) << std::endl;
			return distance_point_to_line;

		}

		inline std::tuple<double, Eigen::Vector2d> distancePointLine2(const Eigen::Vector2d& point, const Eigen::Vector2d& p1_line, const Eigen::Vector2d& p2_line){

			Eigen::Vector2d AB = p2_line - p1_line;
			Eigen::Vector2d AC = point - p1_line;
			Eigen::Vector2d BC = point - p2_line;

			Eigen::Vector2d CCp = Eigen::Vector2d::Zero();
			//Check if closest segment point is on the line segment of the wall
//			First, check to see if the nearest point on the line AB is beyond B (as in the example above) by taking AB ⋅ BC. If this value is greater than 0, it means that the angle between AB and BC is between -90 and 90, exclusive, and therefore the nearest point on the segment AB will be B
//			if(AB.dot(BC) >= 0){
//				CCp = -BC;
//			}else if( (-AB).dot(AC) >= 0 ){
//				CCp = -AC;
//			}else {
				//Vector to line
				double a1 = AC.dot(AB / AB.norm());
				Eigen::Vector2d ACp = a1 * (AB / AB.norm());
				CCp = -AC + ACp;
//			}

//			std::cout << "CCP " << CCp << std::endl;

			return std::make_tuple(CCp.norm(), CCp);

		}

		//Return the distance and the vector from the point to the line
		inline std::tuple<double, Eigen::Vector2d> distancePointSegment(const Eigen::Vector2d& point, const Eigen::Vector2d& p1_line, const Eigen::Vector2d& p2_line){
			Eigen::Vector2d AB = p2_line - p1_line;
			Eigen::Vector2d AC = point - p1_line;
			Eigen::Vector2d BC = point - p2_line;

			Eigen::Vector2d CCp = Eigen::Vector2d::Zero();
			//Check if closest segment point is on the line segment of the wall
//			First, check to see if the nearest point on the line AB is beyond B (as in the example above) by taking AB ⋅ BC. If this value is greater than 0, it means that the angle between AB and BC is between -90 and 90, exclusive, and therefore the nearest point on the segment AB will be B
			if(AB.dot(BC) >= 0){
				CCp = -BC;
			}else if( (-AB).dot(AC) >= 0 ){
				CCp = -AC;
			}else {
				//Vector to line
				double a1 = AC.dot(AB / AB.norm());
				Eigen::Vector2d ACp = a1 * (AB / AB.norm());
				CCp = -AC + ACp;
			}

//			std::cout << "CCP " << CCp << std::endl;

			return std::make_tuple(CCp.norm(), CCp);

		}
		
		
		
	}
}

#endif