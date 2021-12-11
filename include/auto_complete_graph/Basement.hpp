#ifndef AUTOCOMPLETEGRAPH_BASEMENT_26102016
#define AUTOCOMPLETEGRAPH_BASEMENT_26102016

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
// #include "das/CornerDetector.hpp"
#include "PriorLoaderInterface.hpp"

namespace AASS
{
	namespace acg
	{

		/**
		 * @brief The class with all the hardcoded first guess things
		 */
		class Basement : public PriorLoaderInterface
		{

		protected:
		public:
			Basement() : PriorLoaderInterface("/home/malcolm/ros_catkin_ws/indigo_ws/src/auto_complete_graph/tests/map_simple1.png")
			{

				_same_point_prior.push_back(cv::Point2f(287, 99));
				_same_point_slam.push_back(cv::Point2f(19.36, 6.25));

				_same_point_prior.push_back(cv::Point2f(287, 42));
				_same_point_slam.push_back(cv::Point2f(19.14, 2.25));
			}
		};

	}
}

#endif