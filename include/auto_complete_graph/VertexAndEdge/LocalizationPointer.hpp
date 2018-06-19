#ifndef AUTOCOMPLETEGRAPH_LOCALIZATIONPOINTER_01052018
#define AUTOCOMPLETEGRAPH_LOCALIZATIONPOINTER_01052018

#include <auto_complete_graph/Localization/Localization.hpp>
#include "EdgeLandmark.hpp"
#include "VertexSE2RobotLocalization.hpp"
//
//namespace g2o{
//	class VertexXYPrior;
//}

namespace AASS{
	namespace acg{

		class LocalizationPointer : public AASS::acg::Localization {

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			//To check if a link between MCL and the landmark was already done using the localization
			std::unordered_set<g2o::VertexPointXYACG*> _prior_linked_using_landmark;

			//Pose of the landmark in the equivalent robot_frame to this mcl frame when it was detected -> used to translate to the corner.
			Eigen::Vector2d original_pose_in_robot_frame = Eigen::Vector2d::Zero();

			Eigen::Vector2d observation;

			Eigen::Matrix3d cov_inverse = Eigen::Matrix3d::Zero();
			double determinant = 0;
			bool inverse_cov_exist = false;

			LocalizationPointer() : vertex_mcl_pose(NULL), edge_observation_from_mcl_pose(NULL){}
			g2o::VertexSE2RobotLocalization* vertex_mcl_pose;
			g2o::EdgeLandmark_malcolm* edge_observation_from_mcl_pose;
//			g2o::VertexSE2RobotPose* vertex_robot_pose;

			Eigen::Vector3d landmarkSeenByMCLInGlobalFrame() {

				Eigen::Vector3d original_pose_in_mcl_frame_3d;
				Eigen::Vector3d mcl_frame_pose_vec = vertex_mcl_pose->localizationInGlobalFrame();
				g2o::SE2 mcl_frame_pose(mcl_frame_pose_vec);
				original_pose_in_mcl_frame_3d << original_pose_in_robot_frame(0), original_pose_in_robot_frame(1), 0;
//				std::cout << "Pose original " << original_pose_in_robot_frame << std::endl;
//				std::cout << "Pose mcl " << mcl_frame_pose.toVector() << std::endl;
				Eigen::Vector3d pose_inglobal_frame;
				translateFromRobotFrameToGlobalFrame(original_pose_in_mcl_frame_3d, mcl_frame_pose, pose_inglobal_frame);
				return pose_inglobal_frame;

			}

		};

	}
}


#endif