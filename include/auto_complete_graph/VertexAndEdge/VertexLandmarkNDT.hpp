#ifndef AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017
#define AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017

#include "ndt_map/ndt_cell.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "EdgeInterfaceMalcolm.hpp"
#include "VertexSE2RobotPose.hpp"
#include "opencv2/opencv.hpp"

namespace AASS {
namespace acg{	
	
class VertexLandmarkNDT: public g2o::VertexPointXY
	{
		public:
			
		//TESTING
		std::vector<boost::shared_ptr< lslgeneric::NDTCell > > cells_that_gave_it_1;
		std::vector<boost::shared_ptr< lslgeneric::NDTCell > > cells_that_gave_it_2;
		g2o::SE2 robotpose_seen_from;
		
		//END OF TESTING
			
			
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeInterfaceMalcolm interface;

		cv::Point2f position;
		cv::KeyPoint keypoint;
// 		cv::Point2d position;
		cv::Mat descriptor;
		std::pair<double, double> angle_direction;
		AASS::acg::VertexSE2RobotPose* first_seen_from;
		
		VertexLandmarkNDT() : first_seen_from(NULL), g2o::VertexPointXY(){};
		
		double getAngleWidth() const {return angle_direction.first;}
		double getDirection() const {return angle_direction.second;}
		void addAngleDirection(double a, double d){
			if (a < 0) a += 2 * M_PI;
			if (d < 0) d += 2 * M_PI;
			angle_direction = std::pair<double, double>(a, d);};
		double getDirectionGlobal() const {
			assert(first_seen_from != NULL);
			auto vec = first_seen_from->estimate().toVector();
// 			std::cout << "Got estimate " << std::endl;
			double angle = vec(2) + angle_direction.second;
			return angle;
		}

	};

}
}
#endif