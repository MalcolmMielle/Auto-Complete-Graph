#ifndef AUTOCOMPLETEGRAPH_EDGELANDMARK_10112017
#define AUTOCOMPLETEGRAPH_EDGELANDMARK_10112017

#include "EdgeSE2PointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace g2o{

	class EdgeLandmark_malcolm : public g2o::EdgeSE2PointXYACG
  {
    public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		AASS::acg::EdgeInterfaceMalcolm interface;

		Eigen::Vector2d original_observation = Eigen::Vector2d::Zero();
// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeLandmark_malcolm() :
		EdgeSE2PointXYACG()
	{};
	  
// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}
	  

};


}
#endif