#ifndef AUTOCOMPLETEGRAPH_EDGENDTCELLOBSERVATION_03072018
#define AUTOCOMPLETEGRAPH_EDGENDTCELLOBSERVATION_03072018

#include "EdgeSE2PointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"

namespace g2o{

	class EdgeNDTCellObservation : public g2o::EdgeSE2PointXYACG
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		EdgeNDTCellObservation() :
				EdgeSE2PointXYACG()
		{};

// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}


	};


}
#endif