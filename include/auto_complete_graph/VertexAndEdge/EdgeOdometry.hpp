#ifndef AUTOCOMPLETEGRAPH_EDGEODOMETRY_10112017
#define AUTOCOMPLETEGRAPH_EDGEODOMETRY_10112017

#include "EdgeSE2ACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace g2o{

	 class EdgeOdometry_malcolm : public g2o::EdgeSE2ACG
  {
    public:
		 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		 AASS::acg::EdgeInterfaceMalcolm interface;
// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeOdometry_malcolm() : g2o::EdgeSE2ACG(){};
// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}
	  

};

}

#endif