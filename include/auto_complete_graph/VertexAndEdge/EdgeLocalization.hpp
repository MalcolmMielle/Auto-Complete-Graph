#ifndef AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018
#define AUTOCOMPLETEGRAPH_EDGELOCALIZATION_07012018

#include "EdgeSE2ACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace g2o{

	 class EdgeLocalization : public g2o::EdgeSE2ACG
  {
    public:
		 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		 AASS::acg::EdgeInterfaceMalcolm interface;
// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeLocalization() : g2o::EdgeSE2ACG(){};
// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}
	  

};


}
#endif
