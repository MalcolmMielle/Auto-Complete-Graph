#ifndef AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017
#define AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017

#include "ndt_map/ndt_cell.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "EdgeInterfaceMalcolm.hpp"
#include "VertexSE2RobotPose.hpp"
#include "opencv2/opencv.hpp"
#include "VertexSE2Prior.hpp"

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
		double getOrientation() const {return angle_direction.second;}
		void addAngleOrientation(double a, double d){
			if (a < 0) a += 2 * M_PI;
			if (d < 0) d += 2 * M_PI;
			angle_direction = std::pair<double, double>(a, d);};
		double getOrientationGlobal() const {
			assert(first_seen_from != NULL);
			auto vec = first_seen_from->estimate().toVector();
// 			std::cout << "Got estimate " << std::endl;
			double angle = vec(2) + angle_direction.second;
			return angle;
		}
		
		bool sameOrientation(const VertexSE2Prior& v){
			double landmark_angle = getAngleWidth();
			double landmark_direction = getOrientationGlobal();
			
			std::cout << "Orientation calc" << std::endl;
			auto angledirection = v.getAngleOrientation();
			for(auto it = angledirection.begin() ; it != angledirection.end() ; ++it){
				
				double angle_width = it->first;
				double direction = it->second;
				assert(angle_width >= 0.08);
				std::cout << " Angles " << angle_width << " " << direction << " and " << landmark_angle << " " << landmark_direction << std::endl;
				std::cout <<  direction << " <= " << landmark_direction + 0.785 << " && " << direction << " >= " << landmark_direction - 0.785 << " && " <<	angle_width << " <= " << landmark_angle + 0.785 << " && " << angle_width << " >= " << landmark_angle - 0.349 << std::endl;
				
				double landmark_direction_over = landmark_direction + 0.785;
				if (landmark_direction_over < 0) landmark_direction_over += 2 * M_PI;
				double landmark_direction_under = landmark_direction - 0.785;
				if (landmark_direction_under < 0) landmark_direction_under += 2 * M_PI;
				double landmark_angle_under = landmark_angle - 0.349;
				if (landmark_angle_under < 0) landmark_angle_under += 2 * M_PI;
				double landmark_angle_over = landmark_angle + 0.785;
				if (landmark_angle_over >= 2 * M_PI) landmark_angle_over -= 2 * M_PI;
				
				if( direction <= landmark_direction_over && direction >= landmark_direction_under &&
					angle_width <= landmark_angle_over && angle_width >= landmark_angle_under
				){
		// 				if( direction <= landmark_direction + 0.785 && direction >= landmark_direction - 0.785
		// 				){
					std::cout << "Good angle" << std::endl;
		// 					int a;
		// 					std::cin>>a;
					return true;
				}
			}
		// 			std::cout << "Not good " << std::endl;
		// 			int a;
		// 			std::cin>>a;
			return false;
		}

	};

}
}
#endif