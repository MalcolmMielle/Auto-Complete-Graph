#ifndef AUTOCOMPLETEGRAPH_EDGELANDMARK_10112017
#define AUTOCOMPLETEGRAPH_EDGELANDMARK_10112017

#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "EdgeInterfaceMalcolm.hpp"

namespace AASS {
namespace acg{

	class EdgeLandmark_malcolm : public g2o::EdgeSE2PointXY
  {
    public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		EdgeInterfaceMalcolm interface;
// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeLandmark_malcolm() :
		EdgeSE2PointXY()
	{};
	  
// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}
	  

};

}
}
#endif