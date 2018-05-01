#ifndef AUTOCOMPLETEGRAPH_EDGEPRIOROBSERVATION_01052018
#define AUTOCOMPLETEGRAPH_EDGEPRIOROBSERVATION_01052018

#include "EdgeLandmark.hpp"

namespace g2o{

	class EdgePriorObservation : public g2o::EdgeLandmark_malcolm
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//		AASS::acg::EdgeInterfaceMalcolm interface;

		EdgeLandmark_malcolm* equivalent_landmark_observation_edge = NULL;

//		Eigen::Vector2d original_observation = Eigen::Vector2d::Zero();
// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		EdgePriorObservation(EdgeLandmark_malcolm* equivalent_landmark_observation) : equivalent_landmark_observation_edge(equivalent_landmark_observation),
				g2o::EdgeLandmark_malcolm()
		{};

// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}


	};


}
#endif