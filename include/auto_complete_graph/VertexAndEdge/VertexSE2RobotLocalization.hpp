#ifndef AUTOCOMPLETEGRAPH_VERTEXSE2ROBOTLOCALIZATION_06042018
#define AUTOCOMPLETEGRAPH_VERTEXSE2ROBOTLOCALIZATION_06042018

#include "EdgeInterfaceMalcolm.hpp"
#include "VertexSE2ACG.hpp"
#include "VertexSE2RobotPose.hpp"
#include "auto_complete_graph/utils.hpp"
#include "ndt_map/ndt_map.h"
#include "opencv2/core/core.hpp"

#include "VertexNDTCell.hpp"

namespace g2o {

class VertexSE2RobotLocalization : public g2o::VertexSE2RobotPose {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSE2RobotLocalization(const Eigen::Vector3d& to_localization)
        : g2o::VertexSE2RobotPose(), _to_robot_localization(to_localization) {}

   protected:
    Eigen::Matrix3d cov;
    int _index_graphmap;

    Eigen::Vector3d _to_robot_localization;

   public:
    std::vector<VertexNDTCell*> ndt_cells;
    g2o::SE2 initial_noisy_estimate;

    uint32_t time_sec = 0;
    uint32_t time_nsec = 0;

    void setRobotLocalization(const Eigen::Vector3d& loc) {
        _to_robot_localization = loc;
    }

    // Localization pose once optimized
    Eigen::Vector3d localizationInGlobalFrame() const {
        Eigen::Vector3d robot_frame_pose = this->estimate().toVector();
        assert(_to_robot_localization != Eigen::Vector3d::Zero());
        Eigen::Vector3d mcl_pose_inglobal_frame;
        AASS::acg::translateFromRobotFrameToGlobalFrame(
            _to_robot_localization, robot_frame_pose, mcl_pose_inglobal_frame);
        return mcl_pose_inglobal_frame;
    }
    const Eigen::Matrix3d& getCovariance() const { return cov; }
    void setCovariance(const Eigen::Matrix3d& setter) { cov = setter; }
};

}  // namespace g2o
#endif