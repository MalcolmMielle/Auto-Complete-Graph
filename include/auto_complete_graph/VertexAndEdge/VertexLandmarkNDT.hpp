#ifndef AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017
#define AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017

#include <auto_complete_graph/Localization/Localization.hpp>
#include "ndt_map/ndt_cell.h"
#include "VertexPointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "VertexSE2RobotPose.hpp"
#include "opencv2/opencv.hpp"
#include "VertexSE2Prior.hpp"
#include "VertexSE2RobotLocalization.hpp"
#include "auto_complete_graph/utils.hpp"
#include "VertexPointXYACG.hpp"
#include "registration.hpp"
#include "EdgeLandmark.hpp"

#include "LocalizationPointer.hpp"
//
//namespace g2o{
//	class VertexXYPrior;
//}

//namespace AASS{
//	namespace acg{
//
//		class LocalizationPointer : public AASS::acg::Localization {
//
//		public:
//			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//			//To check if a link between MCL and the landmark was already done using the localization
//			std::unordered_set<g2o::VertexPointXYACG*> _prior_linked_using_landmark;
//
//			//Pose of the landmark in the equivalent robot_frame to this mcl frame when it was detected -> used to translate to the corner.
//			Eigen::Vector2d original_pose_in_robot_frame = Eigen::Vector2d::Zero();
//
//			Eigen::Vector2d observation;
//
//			Eigen::Matrix3d cov_inverse = Eigen::Matrix3d::Zero();
//			double determinant = 0;
//			bool inverse_cov_exist = false;
//
//			LocalizationPointer() : vertex_mcl_pose(NULL), edge_observation_from_mcl_pose(NULL){}
//			g2o::VertexSE2RobotLocalization* vertex_mcl_pose;
//			g2o::EdgeLandmark_malcolm* edge_observation_from_mcl_pose;
////			g2o::VertexSE2RobotPose* vertex_robot_pose;
//
//			Eigen::Vector3d landmarkSeenByMCLInGlobalFrame() {
//
//				Eigen::Vector3d original_pose_in_mcl_frame_3d;
//				Eigen::Vector3d mcl_frame_pose_vec = vertex_mcl_pose->localizationInGlobalFrame();
//				g2o::SE2 mcl_frame_pose(mcl_frame_pose_vec);
//				original_pose_in_mcl_frame_3d << original_pose_in_robot_frame(0), original_pose_in_robot_frame(1), 0;
//				std::cout << "Pose original " << original_pose_in_robot_frame << std::endl;
//				std::cout << "Pose mcl " << mcl_frame_pose.toVector() << std::endl;
//				Eigen::Vector3d pose_inglobal_frame;
//				translateFromRobotFrameToGlobalFrame(original_pose_in_mcl_frame_3d, mcl_frame_pose, pose_inglobal_frame);
//				return pose_inglobal_frame;
//
//			}
//
//		};
//
//	}
//}


namespace g2o{
	
	class VertexLandmarkNDT: public g2o::VertexPointXYACG
	{
		public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


		std::unordered_set<std::shared_ptr<AASS::acg::LocalizationPointer> > associated_localization;

		//Covariance of localization
//		std::set<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> > cov_mcl_localization;
		//Covariance of localization
//		Eigen::Matrix2d cov_mcl_localization_inverse;
//		double determinant;
//		bool inverse_cov_exist;
			
		//TESTING
		std::vector<boost::shared_ptr< perception_oru::NDTCell > > cells_that_gave_it_1;
		std::vector<boost::shared_ptr< perception_oru::NDTCell > > cells_that_gave_it_2;
		g2o::SE2 robotpose_seen_from;

		
		//END OF TESTING
		AASS::acg::EdgeInterfaceMalcolm interface;

		cv::Point2f position;
		cv::KeyPoint keypoint;
// 		cv::Point2d position_in_robot_frame;
		cv::Mat descriptor;
		std::vector < std::pair<double, double> > angle_orientation;
		g2o::VertexSE2RobotPose* first_seen_from;
		
		VertexLandmarkNDT() : first_seen_from(NULL), g2o::VertexPointXYACG(){};

		void addLocalization(VertexSE2RobotLocalization* vertex, g2o::EdgeLandmark_malcolm* edge_observation_from_mcl_pose, const Eigen::Vector3d& mean, const Eigen::Matrix3d& cov, const Eigen::Vector2d& observation, const Eigen::Vector2d& original_pose_in_robot_frame, int index){

//			assert(vertex->getEquivalentRobotPose() == vertex_robot_pose);

			std::shared_ptr<AASS::acg::LocalizationPointer> lp (new AASS::acg::LocalizationPointer() );
			lp->vertex_mcl_pose = vertex;
			lp->edge_observation_from_mcl_pose = edge_observation_from_mcl_pose;
//			lp->vertex_robot_pose = vertex_robot_pose;
			lp->mean = mean;
			lp->cov = cov;
			lp->index = index;
			lp->observation = observation;
			lp->original_pose_in_robot_frame = original_pose_in_robot_frame;
			cov.computeInverseAndDetWithCheck(lp->cov_inverse, lp->determinant, lp->inverse_cov_exist);
			if (!lp->inverse_cov_exist) {
				std::cout << "Covariance " << cov << std::endl;
				std::cout << "determinant " << lp->determinant << std::endl;
//				throw std::runtime_error("Inverse covariance not findable");
				std::cout << "Inverse covariance not findable switching ot a small covariance instead" << std::endl;
				Eigen::Matrix3d temp_cov;
				temp_cov << 0.1, 0, 0,
						    0, 0.1, 0,
						    0, 0, 0.1;
				lp->cov = temp_cov;
				cov.computeInverseAndDetWithCheck(lp->cov_inverse, lp->determinant, lp->inverse_cov_exist);
				std::cout << "Did we find the inverse covariance now : " << lp->cov_inverse << std::endl;
				assert(lp->inverse_cov_exist == true);

			}
			associated_localization.insert(lp);
		}

		const Eigen::Matrix3d& getCovarianceObservation(const VertexSE2RobotLocalization* vertex) const {
			auto result = std::find_if(associated_localization.begin(), associated_localization.end(),
			                           [vertex](const std::shared_ptr<AASS::acg::LocalizationPointer> &a)->bool { return a->vertex_mcl_pose == vertex; } );
			return (*result)->cov;
		}
//		Eigen::Matrix2d& getCovarianceObservation(const VertexSE2RobotLocalization* vertex){
//			auto result = std::find_if(associated_localization.begin(), associated_localization.end(),
//			                           [vertex](const AASS::acg::LocalizationPointer &a)->bool { return a.vertex == vertex; } );
//			return result->cov;
//		}
		const Eigen::Matrix3d& getInverseCovarianceObservation(const VertexSE2RobotLocalization* vertex) const {
			auto result = std::find_if(associated_localization.begin(), associated_localization.end(),
			                           [vertex](const std::shared_ptr<AASS::acg::LocalizationPointer> &a)->bool { return a->vertex_mcl_pose == vertex; } );
			return (*result)->cov_inverse;
		}
//		Eigen::Matrix2d& getInverseCovarianceObservation(const VertexSE2RobotLocalization* vertex){
//			auto result = std::find_if(associated_localization.begin(), associated_localization.end(),
//			                           [vertex](const AASS::acg::LocalizationPointer &a)->bool { return a.vertex == vertex; } );
//			return result->cov_inverse;
//		}

		const std::unordered_set<std::shared_ptr<AASS::acg::LocalizationPointer> >& getLocalization() const {return associated_localization;}

//		void setCovarianceObservation(const Eigen::Matrix2d& cov){
//			cov_mcl_localization = cov;
//			//I'm not to sure but I'm stealing this from localization
//			cov.computeInverseAndDetWithCheck(cov_mcl_localization_inverse, determinant, inverse_cov_exist);
//			if (!inverse_cov_exist) {
//				throw std::runtime_error("Inverse covariance not findable");
//			}
//		}

		const std::vector < std::pair<double, double> >& getAnglesAndOrientations() const {return angle_orientation;}
		double getAngleWidth(int i) const {return angle_orientation[i].first;}
		double getOrientation(int i) const {return angle_orientation[i].second;}
// 		double getOrientation() const {return angle_orientation.second;}
		void addAngleOrientation(double a, double d){
			if (a < 0) a += 2 * M_PI;
			if (d < 0) d += 2 * M_PI;
			angle_orientation.push_back(std::pair<double, double>(a, d));};
		void addAnglesOrientations(const std::vector<double>& angles, const std::vector<double>& orientations){
			assert(angles.size() == orientations.size());
			for(int i = 0 ; i < angles.size(); ++i)
			{
				angle_orientation.push_back(std::pair<double, double>(angles[i], orientations[i]));
			}
		}
		double getOrientationGlobal(int i) const {
			assert(first_seen_from != NULL);
			auto vec = first_seen_from->estimate().toVector();
// 			std::cout << "Got estimate " << std::endl;
			double angle = vec(2) + angle_orientation[i].second;
			if(angle >= 2 * M_PI){
				angle = angle - 2 * M_PI;
			}
			else if(angle < 0){
				angle = angle + 2 * M_PI;
			}
			return angle;
		}
		
		bool sameOrientation(std::vector<std::pair<double, double> > angles_orientations_v) const {
			
//			auto angles_orientations_v = v.getAnglesAndOrientations();
			
			for(auto it_ao_v = angles_orientations_v.begin(); it_ao_v != angles_orientations_v.end() ; ++it_ao_v){
				double angle_width_v = it_ao_v->first;
				double orientation_v = it_ao_v->second;
				assert(angle_width_v >= 0.08);
				for(int i_ao = 0; i_ao < getAnglesAndOrientations().size() ; ++i_ao){
					double landmark_angle = getAngleWidth(i_ao);
					double landmark_orientation = getOrientationGlobal(i_ao);
					assert(landmark_orientation >= 0);
					assert(orientation_v >= 0);
					assert(landmark_orientation < 2 * M_PI);
					assert(orientation_v < 2 * M_PI);
					
					//Smallest angle difference
					double orientation_diff = orientation_v - landmark_orientation;
					orientation_diff = fmod( (orientation_diff + M_PI), (2 * M_PI) ) - M_PI;
					
					double angle_diff = std::abs(angle_width_v - landmark_angle);
					
					//They are close enough in orientation
					if( orientation_diff <= M_PI / 4){
//						std::cout << "Same Orientation" << std::endl;
						if(angle_diff <= M_PI / 4){
//							std::cout << "Good angle" << std::endl;
							return true;
						}
					}
					
					//ATTENTION magic numbers here.
					//Landmark orientation + 45deg
// 					double landmark_orientation_over = landmark_orientation + 0.785;
// 					if (landmark_orientation_over < 0) landmark_orientation_over += 2 * M_PI;
// 					//Landmark orientation - 45deg
// 					double landmark_orientation_under = landmark_orientation - 0.785;
// 					if (landmark_orientation_under < 0) landmark_orientation_under += 2 * M_PI;
// 					//Landmark angle - 20deg
// 					double landmark_angle_under = landmark_angle - 0.349;
// 					if (landmark_angle_under < 0) landmark_angle_under += 2 * M_PI;
// 					//Landmark angle + 20deg
// 					double landmark_angle_over = landmark_angle + 0.785;
// 					if (landmark_angle_over >= 2 * M_PI) landmark_angle_over -= 2 * M_PI;
// 					
// 					//BUG Those comparisons are wrong in some cases because the angles are modulo numbers.
// 					if( orientation_v <= landmark_orientation_over && orientation_v >= landmark_orientation_under){
// // 						if (angle_width_v <= landmark_angle_over && angle_width_v >= landmark_angle_under){
// 							std::cout << "Good angle" << std::endl;
// 							return true;
// // 						}
// 					}
				}
				
			}
			return false;
// 			}
// 			double landmark_angle = getAngleWidth();
// 			double landmark_direction = getOrientationGlobal();
// 			
// 			std::cout << "Orientation calc" << std::endl;
// 			auto angledirection = v.getAngleOrientation();
// 			for(auto it = angledirection.begin() ; it != angledirection.end() ; ++it){
// 				
// 				double angle_width = it->first;
// 				double direction = it->second;
// 				assert(angle_width >= 0.08);
// 				std::cout << " Angles " << angle_width << " " << direction << " and " << landmark_angle << " " << landmark_direction << std::endl;
// 				std::cout <<  direction << " <= " << landmark_direction + 0.785 << " && " << direction << " >= " << landmark_direction - 0.785 << " && " <<	angle_width << " <= " << landmark_angle + 0.785 << " && " << angle_width << " >= " << landmark_angle - 0.349 << std::endl;
// 				
// 				double landmark_direction_over = landmark_direction + 0.785;
// 				if (landmark_direction_over < 0) landmark_direction_over += 2 * M_PI;
// 				double landmark_direction_under = landmark_direction - 0.785;
// 				if (landmark_direction_under < 0) landmark_direction_under += 2 * M_PI;
// 				double landmark_angle_under = landmark_angle - 0.349;
// 				if (landmark_angle_under < 0) landmark_angle_under += 2 * M_PI;
// 				double landmark_angle_over = landmark_angle + 0.785;
// 				if (landmark_angle_over >= 2 * M_PI) landmark_angle_over -= 2 * M_PI;
// 				
// 				if( direction <= landmark_direction_over && direction >= landmark_direction_under &&
// 					angle_width <= landmark_angle_over && angle_width >= landmark_angle_under
// 				){
// 		// 				if( direction <= landmark_direction + 0.785 && direction >= landmark_direction - 0.785
// 		// 				){
// 					std::cout << "Good angle" << std::endl;
// 		// 					int a;
// 		// 					std::cin>>a;
// 					return true;
// 				}
// 			}
// 		// 			std::cout << "Not good " << std::endl;
// 		// 			int a;
// 		// 			std::cin>>a;
// 			return false;
		}

	};


}
#endif