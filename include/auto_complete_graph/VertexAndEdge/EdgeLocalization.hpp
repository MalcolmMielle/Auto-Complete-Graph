#ifndef AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018
#define AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018

#include "g2o/types/slam2d/edge_se2.h"
#include "EdgeInterfaceMalcolm.hpp"

namespace AASS {
namespace acg{

	 class EdgeLocalization : public g2o::EdgeSE2
  {
    public:
		 EdgeInterfaceMalcolm interface;
// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeLocalization() : g2o::EdgeSE2(){};
// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}
	  

};

}
}
#endif
