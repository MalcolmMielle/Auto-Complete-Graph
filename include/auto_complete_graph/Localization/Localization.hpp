#ifndef AUTOCOMPLETEGRAPH_LOCALIZATION_09032018
#define AUTOCOMPLETEGRAPH_LOCALIZATION_09032018

//#include <pcl/common/transforms.h>
//#include <ndt_mcl/particle_filter.hpp>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>

#include "auto_complete_graph/LocalizationMsg.h"

namespace AASS {

	namespace acg {


		class Localization {
		public:
			Eigen::Vector3d mean;
			Eigen::Matrix3d cov;
			int index;

			Localization() : index(-1) {};

			auto_complete_graph::LocalizationMsg toMessage() const {
				auto_complete_graph::LocalizationMsg loc_msg;

				geometry_msgs::Point mean_msg;
				tf::pointEigenToMsg(mean, mean_msg);
				loc_msg.mcl_mean = mean_msg;

				std_msgs::Float64MultiArray cov_msg;
				tf::matrixEigenToMsg(cov, cov_msg);
				loc_msg.mcl_cov = cov_msg;

				loc_msg.index.data = index;
				return loc_msg;
			}

			void fromMessage(const auto_complete_graph::LocalizationMsg &loc_msg) {

				tf::pointMsgToEigen(loc_msg.mcl_mean, mean);

				assert(loc_msg.mcl_cov.data.size() == 9);
				for (size_t i = 0; i < 3; ++i) {
					for (size_t j = 0; j < 3; ++j) {
						cov(i, j) = loc_msg.mcl_cov.data[(3 * i) + j];
					}
				}

				index = loc_msg.index.data;

			};
		};


	}
}

#endif