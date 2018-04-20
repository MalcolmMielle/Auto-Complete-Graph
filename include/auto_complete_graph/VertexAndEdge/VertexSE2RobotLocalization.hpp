#ifndef AUTOCOMPLETEGRAPH_VERTEXSE2ROBOTLOCALIZATION_06042018
#define AUTOCOMPLETEGRAPH_VERTEXSE2ROBOTLOCALIZATION_06042018

#include "VertexSE2ACG.hpp"
#include "VertexSE2RobotPose.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "opencv2/core/core.hpp"
#include "ndt_map/ndt_map.h"

namespace g2o{

	class VertexSE2RobotLocalization : public g2o::VertexSE2ACG
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		VertexSE2RobotLocalization() :  _index_graphmap(-1), g2o::VertexSE2ACG(){}

	protected:
		std::shared_ptr<perception_oru::NDTMap> _map;
		Eigen::Affine3d _T;
		Eigen::Matrix3d cov;
		double _time;
		int _index_graphmap;
		g2o::VertexSE2RobotPose* _equivalent_robot_pose = NULL;

	public:
		g2o::SE2 initial_transfo;
		cv::Mat img;

		std::shared_ptr<perception_oru::NDTMap>& getMap(){return _map;}
		const std::shared_ptr<perception_oru::NDTMap>& getMap() const {return _map;}
		void setMap(const std::shared_ptr<perception_oru::NDTMap>& map){_map = map;}
		Eigen::Affine3d getPose(){return _T;}
		const Eigen::Affine3d& getPose() const {return _T;}
		void setPose(const Eigen::Affine3d& T) {_T = T;}
		void setTime(double t){_time = t;}
		double getTime(){return _time;}

		void setEquivalentRobotPose(g2o::VertexSE2RobotPose* eq){_equivalent_robot_pose = eq;}
		g2o::VertexSE2RobotPose* getEquivalentRobotPose(){return _equivalent_robot_pose;}

		void setIndexGraphMap(int i){_index_graphmap = i;}
		int getIndexGraphMap() const {return _index_graphmap;}

		const Eigen::Matrix3d& getCovariance() const {return cov ;}
		void setCovariance(const Eigen::Matrix3d& setter) {cov = setter;}


	};


}
#endif