#ifndef AUTOCOMPLETEGRAPH_EDGEXYPRIOR_17042018
#define AUTOCOMPLETEGRAPH_EDGEXYPRIOR_17042018

#include "EdgePointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace  g2o {

	class VertexXYPrior;
}


namespace g2o{


	class EdgeXYPriorACG : public g2o::EdgePointXYACG
	{
	public:

//		bool use_angle_error = false;
//		bool use_distance_error = true;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		AASS::acg::EdgeInterfaceMalcolm interface;
		EdgeXYPriorACG() : g2o::EdgePointXYACG(){}
		Eigen::Vector2d getOrientation2D(const g2o::VertexXYPrior& from) const;

//		void computeError();

	};


}
#endif