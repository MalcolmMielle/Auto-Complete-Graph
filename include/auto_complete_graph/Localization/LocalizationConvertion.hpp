#ifndef AUTOCOMPLETEGRAPH_LOCALIZATIONCONVERTION_21032018
#define AUTOCOMPLETEGRAPH_LOCALIZATIONCONVERTION_21032018

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>

#include "auto_complete_graph/Localization/Localization.hpp"
#include "auto_complete_graph/LocalizationMsg.h"

namespace AASS {

namespace acg {

auto_complete_graph::LocalizationMsg toMessage(const Localization& loca) {
    auto_complete_graph::LocalizationMsg loc_msg;

    geometry_msgs::Point mean_msg;
    tf::pointEigenToMsg(loca.mean, mean_msg);
    loc_msg.mcl_mean = mean_msg;

    std_msgs::Float64MultiArray cov_msg;
    tf::matrixEigenToMsg(loca.cov, cov_msg);
    loc_msg.mcl_cov = cov_msg;

    loc_msg.index.data = loca.index;
    return loc_msg;
}

void fromMessage(const auto_complete_graph::LocalizationMsg& loc_msg,
                 Localization& loca) {
    tf::pointMsgToEigen(loc_msg.mcl_mean, loca.mean);

    assert(loc_msg.mcl_cov.data.size() == 9);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            loca.cov(i, j) = loc_msg.mcl_cov.data[(3 * i) + j];
        }
    }

    loca.index = loc_msg.index.data;
};

}  // namespace acg
}  // namespace AASS
#endif