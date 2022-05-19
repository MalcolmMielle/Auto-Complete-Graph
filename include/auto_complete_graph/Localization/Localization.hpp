#ifndef AUTOCOMPLETEGRAPH_LOCALIZATION_09032018
#define AUTOCOMPLETEGRAPH_LOCALIZATION_09032018

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>

namespace AASS {

namespace acg {

class Localization {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d mean;
    Eigen::Matrix3d cov;
    int index;

    Localization() : index(-1) { cov(0, 0) = -1; }
};

}  // namespace acg
}  // namespace AASS

#endif