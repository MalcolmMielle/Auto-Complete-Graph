#ifndef AUTOCOMPLETEGRAPH_CONVERSION_22072016
#define AUTOCOMPLETEGRAPH_CONVERSION_22072016

#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_feature/utils.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

namespace AASS{
namespace acg{
	
	
	inline Eigen::Isometry2d Affine3d2Isometry2d(const Eigen::Affine3d& affine) {
		Eigen::Affine2d affine2d = ndt_feature::eigenAffine3dTo2d(affine);
		Eigen::Isometry2d isometry2d;
		isometry2d.translation() = affine2d.translation();
		isometry2d.linear() = affine2d.rotation();
		return isometry2d;
	}
	
// 	inline Eigen::Isometry2d isometry2d2Affine3d(const Eigen::Isometry2d& affine) {
// 		
// 		
// 		
// 		Eigen::Affine2d affine2d = lslgeneric::eigenAffine3dTo2d(affine);
// 		Eigen::Isometry2d isometry2d;
// 		isometry2d.translation() = affine2d.translation();
// 		isometry2d.linear() = affine2d.rotation();
// 		return isometry2d;
// 	}

	inline g2o::SE2 NDTFeatureLink2EdgeSE2(const ndt_feature::NDTFeatureLink& link){
		Eigen::Affine3d affine = link.getRelPose();		
		Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 se2(isometry2d);
		return se2;	
	}
	

	
}
}

#endif