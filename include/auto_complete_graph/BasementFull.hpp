#ifndef AUTOCOMPLETEGRAPH_BASEMENTFULL_15112016
#define AUTOCOMPLETEGRAPH_BASEMENTFULL_15112016

#include <random>

#include "PriorLoaderInterface.hpp"
#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"

namespace AASS {
namespace acg {

/**
 * @brief The class with all the hardcoded first guess things
 */
class BasementFull : public PriorLoaderInterface {
   protected:
   public:
    BasementFull(double deviation,
                 double anglet,
                 double scalet,
                 cv::Point2f center)
        : PriorLoaderInterface(
              "/home/malcolm/ros_catkin_ws/lunar_ws/src/auto_complete_graph/"
              "tests/emergbasement_flipped_nodoor.png") {
        std::vector<cv::Point2f> pt_slam;
        std::vector<cv::Point2f> pt_prior;

        pt_slam.push_back(cv::Point2f(19.36, 6.25));
        pt_prior.push_back(cv::Point2f(786, 373));

        pt_slam.push_back(cv::Point2f(19.14, 2.25));
        pt_prior.push_back(cv::Point2f(788, 311));

        initialize(pt_slam, pt_prior, deviation, anglet, scalet, center);
    }
};

}  // namespace acg
}  // namespace AASS

#endif
