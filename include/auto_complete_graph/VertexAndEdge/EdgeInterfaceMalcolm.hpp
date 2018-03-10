#ifndef AUTOCOMPLETEGRAPH_EDGEINTERFACE_10112017
#define AUTOCOMPLETEGRAPH_EDGEINTERFACE_10112017

#include "g2o/types/slam2d/se2.h"

namespace AASS {
namespace acg{
	
class EdgeInterfaceMalcolm
  {
    public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	  g2o::SE2 _malcolm_original_value;
	  double _malcolm_age;
	  
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeInterfaceMalcolm() : _malcolm_age(1){};
	  
	  virtual g2o::SE2 getOriginalValue(){return _malcolm_original_value;}
	  virtual void setOriginalValue(const g2o::SE2& orig_val){_malcolm_original_value = orig_val;}
	  virtual double getAge(){return _malcolm_age;}
	  virtual void setAge(double a){_malcolm_age = a;}
	  

};

}
}

#endif