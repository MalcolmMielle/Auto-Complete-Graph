#ifndef AUTOCOMPLETEGRAPH_VERTEXSE2ROBOTLOCALIZATION_06042018
#define AUTOCOMPLETEGRAPH_VERTEXSE2ROBOTLOCALIZATION_06042018

#include "VertexSE2ACG.hpp"
#include "VertexSE2RobotPose.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "opencv2/core/core.hpp"
#include "ndt_map/ndt_map.h"
#include "auto_complete_graph/utils.hpp"

#include "VertexNDTCell.hpp"

namespace g2o{

	class VertexSE2RobotLocalization : public g2o::VertexSE2RobotPose
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		VertexSE2RobotLocalization(const Eigen::Vector3d& to_localization) : g2o::VertexSE2RobotPose(), _to_robot_localization(to_localization){}

	protected:
//		std::shared_ptr<perception_oru::NDTMap> _map;
//		Eigen::Affine3d _T;
		Eigen::Matrix3d cov;
//		double _time;
		int _index_graphmap;
//		g2o::VertexSE2RobotPose* _equivalent_robot_pose = NULL;

		Eigen::Vector3d _to_robot_localization;

	public:

		std::vector<VertexNDTCell*> ndt_cells;
		g2o::SE2 initial_noisy_estimate;

		uint32_t time_sec = 0;
		uint32_t time_nsec = 0;

//		g2o::SE2 initial_transfo;
//		cv::Mat img;

//		std::shared_ptr<perception_oru::NDTMap>& getMap(){return _map;}
//		const std::shared_ptr<perception_oru::NDTMap>& getMap() const {return _map;}
//		void setMap(const std::shared_ptr<perception_oru::NDTMap>& map){_map = map;}
//		Eigen::Affine3d getPose(){return _T;}
//		const Eigen::Affine3d& getPose() const {return _T;}
//		void setPose(const Eigen::Affine3d& T) {_T = T;}
//		void setTime(double t){_time = t;}
//		double getTime(){return _time;}

		void setRobotLocalization(const Eigen::Vector3d& loc){
			_to_robot_localization = loc;
		}

		//Localization pose once optimized
		Eigen::Vector3d localizationInGlobalFrame() const {

//			Eigen::Vector3d original_pose_in_robot_frame_3d;
			Eigen::Vector3d robot_frame_pose = this->estimate().toVector();
//			std::cout << "Pose original " << original_pose_in_robot_frame << std::endl;
//			std::cout << "Pose mcl " << mcl_frame_pose.toVector() << std::endl;

			assert(_to_robot_localization != Eigen::Vector3d::Zero());

			Eigen::Vector3d mcl_pose_inglobal_frame;
			AASS::acg::translateFromRobotFrameToGlobalFrame(_to_robot_localization, robot_frame_pose, mcl_pose_inglobal_frame);
			return mcl_pose_inglobal_frame;

		}

		//Unoptimized localizatino pose. Needed because the mapping doesn't optimize it's map.
//		Eigen::Vector3d originalLocalization


//		void setEquivalentRobotPose(g2o::VertexSE2RobotPose* eq){_equivalent_robot_pose = eq;}
//		g2o::VertexSE2RobotPose* getEquivalentRobotPose(){return _equivalent_robot_pose;}

//		void setIndexGraphMap(int i){_index_graphmap = i;}
//		int getIndexGraphMap() const {return _index_graphmap;}

		const Eigen::Matrix3d& getCovariance() const {return cov ;}
		void setCovariance(const Eigen::Matrix3d& setter) {cov = setter;}


	};


}
#endif